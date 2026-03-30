"""
jetson_fusion_gui.py  —  runs on Jetson Nano
============================================
Receives radar tracks from Pi (UDP:9576 JSON)
Reads LD06 LiDAR from serial (/dev/ttyTHS1)
Fuses both sensors equally, calculates combined TTC
Shows unified PyQtGraph GUI.

USAGE:  python jetson_fusion_gui.py
INSTALL: pip install numpy pyqtgraph PyQt5 pyserial
"""

import sys, os, json, time, math, serial, socket, threading, queue
from collections import deque
from dataclasses import dataclass, field
from typing import Optional, List, Dict

import numpy as np

try:
    import pyqtgraph as pg
    from pyqtgraph.Qt import QtWidgets, QtCore, QtGui
except ImportError:
    print("pip install pyqtgraph PyQt5"); sys.exit(1)

# ══════════════════════════════════════════════════════════════
#  CONFIG
# ══════════════════════════════════════════════════════════════
RADAR_PORT      = 9576          # UDP port receiving JSON from Pi
LIDAR_SERIAL    = "/dev/ttyTHS1"
LIDAR_BAUD      = 230400

# RC car geometry (from ld06_corridor.py)
RC_CAR_WIDTH_M      = 0.191
HALF_WIDTH_M        = RC_CAR_WIDTH_M / 2 + 0.025   # ~0.12 m
CENTER_HALF_WIDTH_M = RC_CAR_WIDTH_M / 2
LOOKAHEAD_M         = 3.0
STOP_DIST_M         = 0.80
SLOW_DIST_M         = 1.80

# Fusion thresholds
TTC_IMMINENT_S  = 1.5
TTC_CAUTION_S   = 3.0
RANGE_IMMINENT  = 0.50
RANGE_CAUTION   = 1.20

# Sensor staleness: if no data received within this many seconds, mark as stale
STALE_S = 0.5

GUI_FPS = 15


# ══════════════════════════════════════════════════════════════
#  LD06 LIDAR READER  (from ld06_corridor.py)
# ══════════════════════════════════════════════════════════════
# LD06 constants — exactly matched to ld06_corridor.py
PKT_LEN          = 47
HDR0             = 0x54
HDR1             = 0x2C
PTS_PER_PKT      = 12
ANGLE_OFFSET_DEG = 180.0  # LiDAR mounted 180 degrees rotated on RC car
Y_AXIS_SIGN      = +1.0   # flip to -1.0 if left/right are swapped
MIN_VALID_M      = 0.08
MAX_VALID_M      = 12.0
WINDOW_SEC       = 0.30   # rolling point-cloud window seconds
FOV_HALF_DEG     = 20.0   # only scan ±20° forward cone (40° total)
LIDAR_CONFIRM_FRAMES = 3    # obstacle must appear this many frames in a row to trigger
LIDAR_CLEAR_FRAMES   = 4    # must be clear this many frames in a row to go back to SAFE
MIN_POINTS_PER_ZONE  = 3    # minimum points in a zone to count as a real detection

def decode_packet(pkt: bytes):
    """Decode LD06 47-byte packet → list of (angle_deg, dist_m)."""
    if len(pkt) != PKT_LEN or pkt[0] != HDR0 or pkt[1] != HDR1:
        return None
    start_angle = (pkt[4] | (pkt[5] << 8)) / 100.0
    end_angle   = (pkt[42] | (pkt[43] << 8)) / 100.0
    d_mm = []
    for i in range(PTS_PER_PKT):
        dist = pkt[6 + 3*i] | (pkt[6 + 3*i + 1] << 8)
        d_mm.append(dist)
    ea = end_angle
    sa = start_angle
    if ea < sa: ea += 360.0
    angles = [sa + (ea - sa) * i / (PTS_PER_PKT - 1) for i in range(PTS_PER_PKT)]
    angles = [(a + ANGLE_OFFSET_DEG) % 360.0 for a in angles]
    d_m = [v / 1000.0 for v in d_mm]
    out = [(a, d) for a, d in zip(angles, d_m) if MIN_VALID_M <= d <= MAX_VALID_M]
    return out if out else None

def polar_to_xy(angle_deg, dist_m):
    """Robot frame: x=forward, y=left — same as ld06_corridor.py."""
    th = math.radians(angle_deg)
    x  = dist_m * math.cos(th)
    y  = dist_m * math.sin(th) * Y_AXIS_SIGN
    return x, y


class LidarReader(threading.Thread):
    """Reads LD06, extracts corridor state (CLEAR/SLOW/STOP) and min distances."""
    def __init__(self):
        super().__init__(daemon=True)
        self._stop_evt = threading.Event()
        # Latest data
        self.points_xy: List  = []   # [(x,y), ...]  in robot frame
        self.fl: Optional[float] = None   # front-left zone min dist
        self.fc: Optional[float] = None   # front-center zone min dist
        self.fr: Optional[float] = None   # front-right zone min dist
        self.lidar_state: str = "NO DATA"
        self.last_update: float = 0
        self._pts_hist: deque = deque()  # (timestamp, x, y)
        self._confirm_count: int = 0     # consecutive frames with obstacle
        self._clear_count: int = 0       # consecutive frames without obstacle
        self._confirmed_state: str = "CLEAR"  # debounced state
        self._lock = threading.Lock()

    def stop(self): self._stop_evt.set()

    def get(self):
        with self._lock:
            return (list(self.points_xy), self.fl, self.fc, self.fr,
                    self.lidar_state, self.last_update)

    def run(self):
        buf = b""
        ser = None
        while not self._stop_evt.is_set():
            try:
                if ser is None:
                    ser = serial.Serial(LIDAR_SERIAL, LIDAR_BAUD, timeout=0.1)
                chunk = ser.read(256)
                if not chunk:
                    continue
                buf += chunk
                while len(buf) >= PKT_LEN:
                    idx = buf.find(bytes([HDR0, HDR1]))
                    if idx == -1:
                        buf = buf[-1:]; break
                    if idx > 0:
                        buf = buf[idx:]
                    if len(buf) < PKT_LEN:
                        break
                    pkt = buf[:PKT_LEN]
                    buf = buf[PKT_LEN:]
                    parsed = decode_packet(pkt)
                    if not parsed:
                        continue
                    self._process(parsed)
            except serial.SerialException as e:
                if ser:
                    try: ser.close()
                    except: pass
                    ser = None
                with self._lock:
                    self.lidar_state = "NO DATA"
                time.sleep(1)
            except Exception:
                time.sleep(0.1)

    def _process(self, parsed):
        now = time.time()
        # Add new points to rolling history — forward cone only
        for ang, dist in parsed:
            # After offset, 0° = forward. Accept only ±FOV_HALF_DEG.
            # Normalise angle to -180..+180
            a = ang % 360.0
            if a > 180.0: a -= 360.0
            if abs(a) > FOV_HALF_DEG:
                continue   # ignore anything not in the forward cone
            x, y = polar_to_xy(ang, dist)
            if x <= 0:
                continue   # safety: discard anything behind
            self._pts_hist.append((now, x, y))
        # Expire old points
        cutoff = now - WINDOW_SEC
        while self._pts_hist and self._pts_hist[0][0] < cutoff:
            self._pts_hist.popleft()

        # Zone assignment — exactly mirrors ld06_corridor.py
        xy = []; fl_vals=[]; fc_vals=[]; fr_vals=[]
        for _, x, y in self._pts_hist:
            if not (0.0 < x <= LOOKAHEAD_M):
                continue
            xy.append((x, y))
            if abs(y) > HALF_WIDTH_M:
                continue   # outside corridor width
            d_fwd = x
            if abs(y) <= CENTER_HALF_WIDTH_M:
                fc_vals.append(d_fwd)
            elif y > 0:
                fl_vals.append(d_fwd)   # y>0 = LEFT
            else:
                fr_vals.append(d_fwd)   # y<0 = RIGHT

        fl = min(fl_vals) if fl_vals else None
        fc = min(fc_vals) if fc_vals else None
        fr = min(fr_vals) if fr_vals else None

        # Require minimum points per zone to count as real detection
        if len(fc_vals) < MIN_POINTS_PER_ZONE: fc = None
        if len(fl_vals) < MIN_POINTS_PER_ZONE: fl = None
        if len(fr_vals) < MIN_POINTS_PER_ZONE: fr = None

        # Derive raw state — center checked first (most critical)
        raw_state = "CLEAR"
        for d in (fc, fl, fr):
            if d is not None and d <= STOP_DIST_M:
                raw_state = "STOP"; break
        if raw_state != "STOP":
            for d in (fc, fl, fr):
                if d is not None and d <= SLOW_DIST_M:
                    raw_state = "SLOW"; break

        # Persistence filter — debounce state changes
        # Only trigger if seen consistently, only clear if consistently clear
        if raw_state != "CLEAR":
            self._confirm_count += 1
            self._clear_count = 0
        else:
            self._clear_count += 1
            self._confirm_count = 0

        if self._confirm_count >= LIDAR_CONFIRM_FRAMES:
            self._confirmed_state = raw_state
        elif self._clear_count >= LIDAR_CLEAR_FRAMES:
            self._confirmed_state = "CLEAR"
        # else: keep previous confirmed state — ignore single-frame blips

        with self._lock:
            self.points_xy = xy
            self.fl, self.fc, self.fr = fl, fc, fr
            self.lidar_state = self._confirmed_state
            self.last_update = now


# ══════════════════════════════════════════════════════════════
#  RADAR RECEIVER  (JSON over UDP from Pi)
# ══════════════════════════════════════════════════════════════
@dataclass
class RadarTrack:
    tid: int; range_m: float; vel_mps: float
    ttc_s: Optional[float]; level: str; cls: str


class RadarReceiver(threading.Thread):
    def __init__(self):
        super().__init__(daemon=True)
        self._stop_evt = threading.Event()
        self.tracks: List[RadarTrack] = []
        self.worst: str = "SAFE"
        self.last_update: float = 0
        self._pts_hist: deque = deque()  # (timestamp, x, y)
        self._confirm_count: int = 0     # consecutive frames with obstacle
        self._clear_count: int = 0       # consecutive frames without obstacle
        self._confirmed_state: str = "CLEAR"  # debounced state
        self._lock = threading.Lock()

    def stop(self): self._stop_evt.set()

    def get(self):
        with self._lock:
            return list(self.tracks), self.worst, self.last_update

    def run(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.settimeout(0.5)
        sock.bind(("0.0.0.0", RADAR_PORT))
        print(f"[RADAR RX] Listening UDP:{RADAR_PORT}")
        while not self._stop_evt.is_set():
            try:
                data, addr = sock.recvfrom(65536)
            except socket.timeout:
                continue
            try:
                msg = json.loads(data.decode())
                tracks = [RadarTrack(
                    tid=t["id"], range_m=t["range_m"], vel_mps=t["vel_mps"],
                    ttc_s=t["ttc_s"], level=t["level"], cls=t["cls"]
                ) for t in msg.get("tracks", [])]
                with self._lock:
                    self.tracks = tracks
                    self.worst  = msg.get("worst", "SAFE")
                    self.last_update = time.time()
            except Exception:
                pass
        sock.close()


# ══════════════════════════════════════════════════════════════
#  SENSOR FUSION
# ══════════════════════════════════════════════════════════════
def lidar_ttc(fc: Optional[float]) -> Optional[float]:
    """Estimate TTC from closest forward LiDAR obstacle (no velocity info)."""
    # LiDAR has no velocity → return range only as a proxy warning distance
    return None  # TTC unavailable without velocity; use range thresholds instead


def lidar_level(fl, fc, fr) -> str:
    """Map LiDAR zone distances to collision level."""
    min_d = min((d for d in [fl, fc, fr] if d is not None), default=None)
    if min_d is None: return "SAFE"
    if min_d <= RANGE_IMMINENT: return "IMMINENT"
    if min_d <= RANGE_CAUTION:  return "CAUTION"
    if min_d <= STOP_DIST_M:    return "IMMINENT"
    if min_d <= SLOW_DIST_M:    return "CAUTION"
    return "SAFE"


LEVEL_RANK = {"SAFE": 0, "CAUTION": 1, "IMMINENT": 2}

def fuse(radar_worst: str, radar_tracks: List[RadarTrack],
         lidar_state: str, fl, fc, fr,
         radar_stale: bool, lidar_stale: bool) -> dict:
    """
    Equal-weight fusion.
    Returns dict with fused_level, fused_ttc_s, detail string.
    """
    lvl_r = radar_worst if not radar_stale else "SAFE"
    lvl_l = lidar_level(fl, fc, fr) if not lidar_stale else "SAFE"

    # Map lidar_state to level as well
    lidar_map = {"STOP": "IMMINENT", "SLOW": "CAUTION",
                 "CLEAR": "SAFE", "NO DATA": "SAFE"}
    lvl_l2 = lidar_map.get(lidar_state, "SAFE")
    # Take worst of the two lidar interpretations
    lvl_l = ["SAFE","CAUTION","IMMINENT"][max(LEVEL_RANK[lvl_l], LEVEL_RANK[lvl_l2])]

    # Equal-weight: take worst of radar and lidar
    fused = ["SAFE","CAUTION","IMMINENT"][max(LEVEL_RANK[lvl_r], LEVEL_RANK[lvl_l])]

    # Best TTC from approaching radar tracks
    best_ttc = None
    for t in radar_tracks:
        if not radar_stale and t.ttc_s is not None:
            best_ttc = t.ttc_s if best_ttc is None else min(best_ttc, t.ttc_s)

    # Min lidar distance
    min_lidar = min((d for d in [fl, fc, fr] if d is not None), default=None)

    detail_parts = []
    if not radar_stale:
        detail_parts.append(f"Radar:{lvl_r}")
        if best_ttc is not None: detail_parts.append(f"TTC={best_ttc:.1f}s")
    else:
        detail_parts.append("Radar:STALE")
    if not lidar_stale:
        detail_parts.append(f"LiDAR:{lidar_state}")
        if min_lidar: detail_parts.append(f"d={min_lidar:.2f}m")
    else:
        detail_parts.append("LiDAR:STALE")

    return {
        "level":     fused,
        "ttc_s":     best_ttc,
        "min_lidar": min_lidar,
        "radar_lvl": lvl_r,
        "lidar_lvl": lvl_l,
        "detail":    "  |  ".join(detail_parts),
        "radar_stale": radar_stale,
        "lidar_stale": lidar_stale,
    }


# ══════════════════════════════════════════════════════════════
#  GUI
# ══════════════════════════════════════════════════════════════
BG    = "#0a0f0a"
GRN   = "#00ff41"; DIMGRN = "#004d14"
AMBER = "#ffb300"; RED    = "#ff2020"
CYAN  = "#00e5ff"; WHITE  = "#e8ffe8"
GREY  = "#556655"

LEVEL_BANNER = {
    "SAFE":     ("◈  ALL CLEAR  ◈",
                 f"background:#003300;color:{GRN};border:3px solid {GRN};"),
    "CAUTION":  ("⚠  CAUTION  ⚠",
                 f"background:#332200;color:{AMBER};border:3px solid {AMBER};"),
    "IMMINENT": ("⛔  COLLISION IMMINENT  ⛔",
                 f"background:#440000;color:{RED};border:3px solid {RED};"),
}

TCOLORS = [(0,255,65),(0,229,255),(255,179,0),(255,80,80),
           (180,80,255),(0,200,150),(255,140,0),(120,220,255)]


class FusionGUI:
    def __init__(self, lidar: LidarReader, radar: RadarReceiver):
        self.lidar = lidar
        self.radar = radar
        self._fc = 0; self._fps = 0.0; self._ft = time.time()
        self._radar_trails: dict = {}
        self._radar_labels: dict = {}

        pg.setConfigOption("background", BG)
        pg.setConfigOption("foreground", GRN)

        self.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication(sys.argv)
        self.win = QtWidgets.QMainWindow()
        self.win.setWindowTitle("Sensor Fusion  ·  LiDAR + 60 GHz Radar  ·  Collision Detection")
        self.win.resize(1600, 900)
        self.win.setStyleSheet(f"background:{BG};color:{GRN};")

        cw = QtWidgets.QWidget(); self.win.setCentralWidget(cw)
        root = QtWidgets.QVBoxLayout(cw)
        root.setContentsMargins(8,6,8,6); root.setSpacing(6)

        # ── FUSION BANNER ──────────────────────────────────────────
        self.banner = QtWidgets.QLabel("◈  ALL CLEAR  ◈")
        self.banner.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self.banner.setFixedHeight(70)
        self.banner.setFont(QtGui.QFont("Courier New", 28, QtGui.QFont.Weight.Bold))
        self._set_banner("SAFE"); root.addWidget(self.banner)

        # ── SENSOR STATUS ROW ─────────────────────────────────────
        status_row = QtWidgets.QHBoxLayout(); status_row.setSpacing(8)
        root.addLayout(status_row)

        self.lbl_radar  = self._make_status_lbl("RADAR  ·  waiting...")
        self.lbl_lidar  = self._make_status_lbl("LiDAR  ·  waiting...")
        self.lbl_fused  = self._make_status_lbl("FUSED  ·  —")
        for lbl in [self.lbl_radar, self.lbl_lidar, self.lbl_fused]:
            status_row.addWidget(lbl)

        # ── THREE PANELS ──────────────────────────────────────────
        panels = QtWidgets.QHBoxLayout(); panels.setSpacing(8)
        root.addLayout(panels, stretch=10)

        # LEFT: LiDAR bird's-eye
        glw_l = pg.GraphicsLayoutWidget()
        self.lp = glw_l.addPlot(title="LiDAR  ·  Corridor View")
        self.lp.titleLabel.setAttr("color", CYAN)
        self.lp.setAspectLocked(True)
        self.lp.setXRange(-0.3, LOOKAHEAD_M+0.2)
        self.lp.setYRange(-HALF_WIDTH_M*3, HALF_WIDTH_M*3)
        self.lp.setLabel("bottom","Forward (m)"); self.lp.setLabel("left","Lateral (m)")
        self._draw_lidar_static()
        self.lidar_sc = pg.ScatterPlotItem(size=4, pen=None)
        self.lp.addItem(self.lidar_sc)
        panels.addWidget(glw_l, stretch=4)

        # MIDDLE: Radar top-down
        glw_r = pg.GraphicsLayoutWidget()
        self.rp = glw_r.addPlot(title="Radar  ·  Range-Velocity")
        self.rp.titleLabel.setAttr("color", GRN)
        self.rp.setLabel("left","Range (m)"); self.rp.setLabel("bottom","Velocity (m/s)")
        self.rp.setXRange(-3, 3); self.rp.setYRange(0, 5)
        # Range rings
        for r in [0.5, 1.0, 1.5, 2.0, 3.0, 5.0]:
            e = QtWidgets.QGraphicsEllipseItem(-r,0,2*r,2*r)
            e.setPen(pg.mkPen(DIMGRN,width=0.6)); e.setBrush(pg.mkBrush(None))
            self.rp.addItem(e)
        for r, col in [(RANGE_CAUTION, AMBER), (RANGE_IMMINENT, RED)]:
            e = QtWidgets.QGraphicsEllipseItem(-r,0,2*r,2*r)
            e.setPen(pg.mkPen(col,width=1.4,style=QtCore.Qt.PenStyle.DashLine))
            e.setBrush(pg.mkBrush(None)); self.rp.addItem(e)
        self.rp.addItem(pg.ScatterPlotItem([{
            "pos":(0,0),"brush":pg.mkBrush(0,229,255,200),
            "pen":pg.mkPen(WHITE,width=1),"size":16,"symbol":"t"}]))
        self.radar_sc = pg.ScatterPlotItem(size=16)
        self.rp.addItem(self.radar_sc)
        panels.addWidget(glw_r, stretch=4)

        # RIGHT: TTC + track table
        right = QtWidgets.QVBoxLayout(); right.setSpacing(6)
        panels.addLayout(right, stretch=3)

        # TTC bar chart
        glw_ttc = pg.GraphicsLayoutWidget(); glw_ttc.setMaximumWidth(380)
        self.ttcp = glw_ttc.addPlot(title="Time-To-Collision")
        self.ttcp.titleLabel.setAttr("color",GRN)
        self.ttcp.hideAxis("left"); self.ttcp.setLabel("bottom","TTC (s)")
        self.ttcp.setXRange(0, TTC_CAUTION_S+0.5); self.ttcp.setYRange(0,8)
        for x,col in [(TTC_IMMINENT_S,RED),(TTC_CAUTION_S,AMBER)]:
            self.ttcp.addLine(x=x,pen=pg.mkPen(col,width=1.5,
                style=QtCore.Qt.PenStyle.DashLine))
        self.ttc_bars:dict={}; self.ttc_txts:dict={}
        right.addWidget(glw_ttc, stretch=5)

        # Track table
        self.table = QtWidgets.QTableWidget(0, 5)
        self.table.setHorizontalHeaderLabels(["Src","Range","Vel","TTC","Level"])
        self.table.horizontalHeader().setStretchLastSection(True)
        self.table.verticalHeader().setVisible(False)
        self.table.setStyleSheet(
            f"background:{BG};color:{GRN};gridline-color:{DIMGRN};"
            f"QHeaderView::section{{background:#0a1f0a;color:{GRN};"
            f"font-family:Courier New;font-size:8pt;}}"
            f"font-family:Courier New;font-size:8pt;")
        right.addWidget(self.table, stretch=5)

        # Status bar
        self.sbar = QtWidgets.QLabel("")
        self.sbar.setFont(QtGui.QFont("Courier New",8))
        self.sbar.setStyleSheet(f"color:{GREY};")
        root.addWidget(self.sbar)

        # Timer
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self._update)
        self.timer.start(1000 // GUI_FPS)
        self.win.show()

    def _make_status_lbl(self, text):
        lbl = QtWidgets.QLabel(text)
        lbl.setFont(QtGui.QFont("Courier New", 9, QtGui.QFont.Weight.Bold))
        lbl.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        lbl.setFixedHeight(32)
        lbl.setStyleSheet(f"background:#0a1f0a;color:{GRN};"
                          f"border:1px solid {DIMGRN};border-radius:3px;padding:2px;")
        return lbl

    def _draw_lidar_static(self):
        """Draw corridor lines and distance markers on LiDAR plot."""
        ax = self.lp
        # Corridor boundaries
        for y in [HALF_WIDTH_M, -HALF_WIDTH_M]:
            line = pg.InfiniteLine(pos=y, angle=0, pen=pg.mkPen(CYAN,width=1.2))
            ax.addItem(line)
        for y in [CENTER_HALF_WIDTH_M, -CENTER_HALF_WIDTH_M]:
            line = pg.InfiniteLine(pos=y, angle=0,
                pen=pg.mkPen(CYAN,width=0.6,style=QtCore.Qt.PenStyle.DotLine))
            ax.addItem(line)
        # STOP/SLOW distance lines (vertical in forward-axis plot)
        for x, col, label in [(STOP_DIST_M, RED, "STOP"), (SLOW_DIST_M, AMBER, "SLOW")]:
            line = pg.InfiniteLine(pos=x, angle=90,
                pen=pg.mkPen(col,width=1.2,style=QtCore.Qt.PenStyle.DashLine))
            ax.addItem(line)
            txt = pg.TextItem(label, color=col, anchor=(0,1))
            txt.setFont(QtGui.QFont("Courier New",8))
            txt.setPos(x, HALF_WIDTH_M)
            ax.addItem(txt)
        # Robot marker
        ax.addItem(pg.ScatterPlotItem([{
            "pos":(0,0),"brush":pg.mkBrush(0,229,255,200),
            "pen":pg.mkPen(WHITE,width=1),"size":16,"symbol":"t"}]))

    def _set_banner(self, level):
        txt, style = LEVEL_BANNER.get(level, LEVEL_BANNER["SAFE"])
        self.banner.setText(txt)
        self.banner.setStyleSheet(f"padding:4px;border-radius:6px;{style}")

    def _set_status(self, lbl, text, ok=True):
        col = GRN if ok else RED
        lbl.setText(text)
        lbl.setStyleSheet(f"background:#0a1f0a;color:{col};"
                          f"border:1px solid {col};border-radius:3px;padding:2px;")

    def _cell(self, text, color=None):
        item = QtWidgets.QTableWidgetItem(text)
        item.setForeground(QtGui.QColor(color or GRN))
        item.setTextAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        return item

    def _update(self):
        now = time.time()

        # ── Pull latest data ───────────────────────────────────────
        pts, fl, fc, fr, lidar_state, lidar_t = self.lidar.get()
        radar_tracks, radar_worst, radar_t     = self.radar.get()

        radar_stale = (now - radar_t) > STALE_S
        lidar_stale = (now - lidar_t) > STALE_S

        # ── Fuse ──────────────────────────────────────────────────
        fusion = fuse(radar_worst, radar_tracks,
                      lidar_state, fl, fc, fr,
                      radar_stale, lidar_stale)
        level = fusion["level"]

        # ── Banner ────────────────────────────────────────────────
        self._set_banner(level)

        # ── Sensor status labels ───────────────────────────────────
        radar_ok = not radar_stale
        lidar_ok = not lidar_stale
        ttc_str  = (f"TTC:{fusion['ttc_s']:.1f}s" if fusion["ttc_s"] else "TTC:—")
        self._set_status(self.lbl_radar,
            f"RADAR  ·  {radar_worst}  ·  {len(radar_tracks)} tracks" +
            ("  [STALE]" if radar_stale else ""), ok=radar_ok)
        self._set_status(self.lbl_lidar,
            f"LiDAR  ·  {lidar_state}" +
            (f"  ·  fwd={fusion['min_lidar']:.2f}m" if fusion["min_lidar"] else "") +
            ("  [STALE]" if lidar_stale else ""), ok=lidar_ok)
        lvl_col = RED if level=="IMMINENT" else (AMBER if level=="CAUTION" else GRN)
        self.lbl_fused.setText(f"FUSED  ·  {level}  ·  {ttc_str}")
        self.lbl_fused.setStyleSheet(
            f"background:#0a1f0a;color:{lvl_col};"
            f"border:1px solid {lvl_col};border-radius:3px;padding:2px;")

        # ── LiDAR plot ────────────────────────────────────────────
        if pts:
            xs = np.array([p[0] for p in pts])
            ys = np.array([p[1] for p in pts])
            # Color by zone
            brushes = []
            for x, y in zip(xs, ys):
                if x <= STOP_DIST_M:              brushes.append(pg.mkBrush(255,32,32,200))
                elif x <= SLOW_DIST_M:            brushes.append(pg.mkBrush(255,179,0,200))
                else:                             brushes.append(pg.mkBrush(0,229,255,160))
            self.lidar_sc.setData(
                [{"pos":(x,y),"brush":b,"size":4} for (x,y),b in zip(zip(xs,ys),brushes)])
        else:
            self.lidar_sc.setData([])

        # ── Radar plot ────────────────────────────────────────────
        active = {t.tid for t in radar_tracks}
        for tid in list(self._radar_trails):
            if tid not in active: self.rp.removeItem(self._radar_trails.pop(tid))
        for tid in list(self._radar_labels):
            if tid not in active: self.rp.removeItem(self._radar_labels.pop(tid))

        spots = []
        for t in radar_tracks:
            col = TCOLORS[t.tid % len(TCOLORS)]
            lp  = (255,32,32) if t.level=="IMMINENT" else ((255,179,0) if t.level=="CAUTION" else (0,255,65))
            sz  = 22 if t.level=="IMMINENT" else (16 if t.level=="CAUTION" else 12)
            x   = np.clip(t.vel_mps * 0.8, -3, 3)
            y   = t.range_m
            spots.append({"pos":(x,y),"brush":pg.mkBrush(*lp,210),
                           "pen":pg.mkPen(*col,width=2),"size":sz})
            ttc_txt = f"{t.ttc_s:.1f}s" if t.ttc_s else "—"
            lbl_txt = f"#{t.tid}  {t.range_m:.2f}m  {t.vel_mps:+.2f}m/s\nTTC:{ttc_txt}"
            if t.tid not in self._radar_labels:
                lb=pg.TextItem(lbl_txt,anchor=(0,1),color=col)
                lb.setFont(QtGui.QFont("Courier New",7))
                self.rp.addItem(lb); self._radar_labels[t.tid]=lb
            self._radar_labels[t.tid].setText(lbl_txt)
            self._radar_labels[t.tid].setPos(x,y)
        self.radar_sc.setData(spots)

        # ── TTC bars (radar tracks only — lidar has no velocity) ──
        for tid in list(self.ttc_bars):
            if tid not in active: self.ttcp.removeItem(self.ttc_bars.pop(tid))
        for tid in list(self.ttc_txts):
            if tid not in active: self.ttcp.removeItem(self.ttc_txts.pop(tid))
        approaching = [t for t in radar_tracks
                       if t.ttc_s is not None and t.cls=="APPROACHING"]
        self.ttcp.setYRange(0, max(len(approaching)*1.5+1, 4))
        for i, t in enumerate(sorted(approaching, key=lambda x: x.ttc_s or 99)):
            y0  = i*1.4+0.2
            ttc = min(t.ttc_s, TTC_CAUTION_S+0.4)
            col = RED if t.level=="IMMINENT" else (AMBER if t.level=="CAUTION" else GRN)
            bar = pg.BarGraphItem(x0=0,x1=ttc,y0=y0,y1=y0+1.0,brush=pg.mkBrush(col+"aa"))
            if t.tid in self.ttc_bars: self.ttcp.removeItem(self.ttc_bars[t.tid])
            self.ttcp.addItem(bar); self.ttc_bars[t.tid]=bar
            ls=f"R#{t.tid} {t.ttc_s:.1f}s"
            if t.tid not in self.ttc_txts:
                tx=pg.TextItem(ls,anchor=(0,0.5),color=(220,220,220))
                tx.setFont(QtGui.QFont("Courier New",8))
                self.ttcp.addItem(tx); self.ttc_txts[t.tid]=tx
            self.ttc_txts[t.tid].setText(ls)
            self.ttc_txts[t.tid].setPos(0.05,y0+0.5)

        # ── Table: all sources ─────────────────────────────────────
        rows = []
        # Radar rows
        for t in sorted(radar_tracks, key=lambda x: x.range_m):
            ttc = f"{t.ttc_s:.1f}s" if t.ttc_s else "—"
            arr = "←" if t.vel_mps<0 else ("→" if t.vel_mps>0 else "·")
            lc  = RED if t.level=="IMMINENT" else (AMBER if t.level=="CAUTION" else GRN)
            rows.append(("📡", f"{t.range_m:.2f}m", f"{t.vel_mps:+.2f}{arr}", ttc, (t.level,lc)))
        # LiDAR row
        for zone, d in [("FL",fl),("FC",fc),("FR",fr)]:
            if d is not None:
                lev = ("IMMINENT" if d<=RANGE_IMMINENT else "CAUTION" if d<=RANGE_CAUTION else "SAFE")
                lc  = RED if lev=="IMMINENT" else (AMBER if lev=="CAUTION" else GRN)
                rows.append(("🔭 "+zone, f"{d:.2f}m", "—", "—", (lev, lc)))

        self.table.setRowCount(len(rows))
        for i,(src,rng,vel,ttc,(lev,lc)) in enumerate(rows):
            self.table.setItem(i,0,self._cell(src))
            self.table.setItem(i,1,self._cell(rng))
            self.table.setItem(i,2,self._cell(vel))
            self.table.setItem(i,3,self._cell(ttc))
            self.table.setItem(i,4,self._cell(lev,lc))
        self.table.resizeColumnsToContents()

        # ── FPS ───────────────────────────────────────────────────
        self._fc += 1
        if now - self._ft >= 1.0:
            self._fps = self._fc / (now - self._ft)
            self._fc = 0; self._ft = now

        self.sbar.setText(
            f"  FPS:{self._fps:.1f}  |  "
            f"Radar tracks:{len(radar_tracks)}  |  "
            f"LiDAR pts:{len(pts)}  |  "
            f"Fusion:{fusion['detail']}  |  "
            f"STOP<{STOP_DIST_M}m  SLOW<{SLOW_DIST_M}m  "
            f"TTC-IMMINENT<{TTC_IMMINENT_S}s  TTC-CAUTION<{TTC_CAUTION_S}s")

    def run(self): self.app.exec()


# ══════════════════════════════════════════════════════════════
#  MAIN
# ══════════════════════════════════════════════════════════════
def main():
    print("=" * 60)
    print("  Sensor Fusion GUI  —  LiDAR + 60 GHz Radar")
    print(f"  Radar UDP   : 0.0.0.0:{RADAR_PORT}")
    print(f"  LiDAR serial: {LIDAR_SERIAL}  @{LIDAR_BAUD}")
    print("=" * 60)

    lidar = LidarReader()
    radar = RadarReceiver()
    lidar.start()
    radar.start()

    print("[INFO] Sensors started. Launching GUI...")
    try:
        FusionGUI(lidar, radar).run()
    except KeyboardInterrupt:
        print("\n[EXIT]")
    finally:
        lidar.stop(); radar.stop()

if __name__ == "__main__":
    main()