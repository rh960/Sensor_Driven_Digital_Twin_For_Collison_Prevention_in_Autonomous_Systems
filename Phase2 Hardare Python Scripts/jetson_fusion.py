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

# Camera
# IMX477 on Jetson Orin Nano JetPack 6 — nvarguscamerasrc confirmed working
CAM_PIPELINE     = ("nvarguscamerasrc sensor-id=0 ! "
                    "video/x-raw(memory:NVMM), width=1920, height=1080, framerate=30/1 ! "
                    "nvvidconv ! video/x-raw, width=416, height=416, format=BGRx ! "
                    "videoconvert ! video/x-raw, format=BGR ! "
                    "queue leaky=downstream max-size-buffers=1 max-size-time=0 max-size-bytes=0 ! "
                    "appsink drop=true max-buffers=1 sync=false emit-signals=false")
CAM_FALLBACK     = 0          # fallback index if GStreamer fails
CAM_WIDTH        = 640
CAM_HEIGHT       = 480
# Centre zone: middle 40% of frame width triggers collision warning
CENTRE_ZONE_FRAC = 0.40       # fraction of frame width considered "directly ahead"
YOLO_CONF        = 0.40       # minimum confidence threshold
YOLO_MODEL       = "yolov8n.pt"
CAM_FPS          = 15         # target display FPS
YOLO_EVERY_N     = 2          # run YOLO every 2nd frame for speed
YOLO_IMG_SIZE    = 416        # YOLO inference resolution (smaller = faster)

# Bounding box height thresholds for relative distance estimation (option 3 — no calibration).
# Based on 416x416 input. A box that fills more of the frame height = closer object.
# Tune these values against your camera/scene after physical mounting.
#   box_h > 200px  → NEAR    (~0–0.8m equivalent)
#   box_h > 100px  → MID     (~0.8–1.8m equivalent)
#   box_h > 40px   → FAR     (~1.8–3.0m equivalent)
#   box_h <= 40px  → DISTANT (>3.0m — beyond collision concern)
CAM_DIST_NEAR_PX    = 200   # box height above this → treat as IMMINENT range
CAM_DIST_MID_PX     = 100   # box height above this → treat as CAUTION range
CAM_DIST_FAR_PX     = 40    # box height above this → object present but distant
# Below CAM_DIST_FAR_PX: detected but too far to be a collision concern

def cam_dist_band(box_h_px: float) -> str:
    """Return distance band label from bounding box height in pixels."""
    if box_h_px >= CAM_DIST_NEAR_PX: return "NEAR"
    if box_h_px >= CAM_DIST_MID_PX:  return "MID"
    if box_h_px >= CAM_DIST_FAR_PX:  return "FAR"
    return "DISTANT"

def cam_dist_level(box_h_px: float) -> str:
    """Map bounding box height to collision level."""
    if box_h_px >= CAM_DIST_NEAR_PX: return "IMMINENT"
    if box_h_px >= CAM_DIST_MID_PX:  return "CAUTION"
    if box_h_px >= CAM_DIST_FAR_PX:  return "CAUTION"
    return "SAFE"   # too distant to be a concern

# COCO classes that YOLOv8n is trained on.
# Any detection whose class name is NOT in this set is labelled "Foreign Object".
# Add or remove entries here to tune what counts as a known/expected class.
KNOWN_CLASSES = {
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train",
    "truck", "boat", "traffic light", "fire hydrant", "stop sign",
    "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep",
    "cow", "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella",
    "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard",
    "sports ball", "kite", "baseball bat", "baseball glove", "skateboard",
    "surfboard", "tennis racket", "bottle", "wine glass", "cup", "fork",
    "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange",
    "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair",
    "couch", "potted plant", "bed", "dining table", "toilet", "tv",
    "laptop", "mouse", "remote", "keyboard", "cell phone", "microwave",
    "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase",
    "scissors", "teddy bear", "hair drier", "toothbrush",
}



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
MIN_VALID_M      = 0.20    # LD06 blind zone — anything under ~20cm is unreliable
MAX_VALID_M      = 12.0
LIDAR_NEARFIELD_M = 0.15   # secondary guard: discard zone readings closer than this
                            # even if MIN_VALID_M passes, near-field optics produce ghosts
WINDOW_SEC       = 0.30   # rolling point-cloud window seconds
FOV_HALF_DEG     = 20.0   # only scan ±20° forward cone (40° total)
LIDAR_CONFIRM_FRAMES  = 3   # obstacle must appear this many frames in a row to trigger
LIDAR_CLEAR_FRAMES    = 4   # must be clear this many frames in a row to go back to SAFE
LIDAR_BLOCKED_FRAMES  = 15  # consecutive zero-point frames → rain/blocked lens
CAM_DARK_THRESHOLD    = 40  # mean frame brightness below this → lens blocked/covered

# Distance-adaptive minimum points per zone.
# Close objects subtend more scan lines so genuine returns are dense — require more points.
# Distant objects produce fewer returns naturally — lower the threshold so they aren't missed.
#   d < 0.5m  → 6 points  (very close, dense returns, ghost tolerance tight)
#   d < 1.0m  → 5 points
#   d < 1.5m  → 4 points
#   d < 2.5m  → 3 points
#   d >= 2.5m → 2 points  (sparse returns at range, accept lower density)
ZONE_MIN_PTS_THRESHOLDS = [
    (0.5,  6),
    (1.0,  5),
    (1.5,  4),
    (2.5,  3),
    (float('inf'), 2),
]

def min_pts_for_dist(dist_m: float) -> int:
    """Return the minimum point count required for a zone at the given distance."""
    for threshold, pts in ZONE_MIN_PTS_THRESHOLDS:
        if dist_m < threshold:
            return pts
    return 2

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
        self._zero_pt_count: int = 0     # consecutive frames with zero valid forward points
        self.lidar_degraded: bool = False # True = rain/blocked (data arriving but no points)
        self._lock = threading.Lock()

    def stop(self): self._stop_evt.set()

    def get(self):
        with self._lock:
            return (list(self.points_xy), self.fl, self.fc, self.fr,
                    self.lidar_state, self.last_update, self.lidar_degraded)

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
            # Near-field ghost filter — discard points suspiciously close to the sensor.
            # The LD06 optics produce spurious returns in the near field even after
            # MIN_VALID_M filtering. These look like real IMMINENT obstacles but aren't.
            dist = math.hypot(x, y)
            if dist < LIDAR_NEARFIELD_M:
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

        # Distance-adaptive minimum points — require more points for close objects
        # (dense genuine returns) and fewer for distant objects (sparse by nature)
        if fc is not None and len(fc_vals) < min_pts_for_dist(fc): fc = None
        if fl is not None and len(fl_vals) < min_pts_for_dist(fl): fl = None
        if fr is not None and len(fr_vals) < min_pts_for_dist(fr): fr = None

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

        # ── LiDAR health / degraded detection ────────────────────────
        # Count raw parsed points (all angles) — if the LiDAR is spinning and
        # the serial stream is healthy, parsed always has points regardless of
        # what direction they face. Zero parsed points = LiDAR not spinning or
        # packet decode failing — different from "blocked".
        total_raw_pts = len(parsed)

        # Forward zone points (xy already filtered to forward corridor).
        # If raw points are arriving but none make it through the forward
        # filter for many frames in a row, the sensor is degraded:
        #   - Lens blocked / dirty → all returns very short or zero
        #   - Rain → scatter pushes all returns below MIN_VALID_M
        #   - Sensor tilted up/down badly → no returns in forward cone
        forward_pts = len(xy)

        if total_raw_pts == 0:
            # No raw points at all — serial OK but LiDAR not spinning or decode broken
            # Mark as stale via last_update not updating rather than degraded
            self._zero_pt_count = 0   # reset — different failure mode
            lidar_degraded = False
        elif forward_pts == 0:
            # Raw points arriving but nothing valid in forward zone — degraded
            self._zero_pt_count += 1
            lidar_degraded = self._zero_pt_count >= LIDAR_BLOCKED_FRAMES
        else:
            self._zero_pt_count = 0
            lidar_degraded = False

        with self._lock:
            self.points_xy = xy
            self.fl, self.fc, self.fr = fl, fc, fr
            self.lidar_state = self._confirmed_state
            self.lidar_degraded = lidar_degraded
            # Only advance last_update when real raw points arrived.
            # Zero raw points means the LiDAR isn't spinning or packets are
            # corrupt — let the stale timer fire so the GUI shows DISCONNECTED.
            if total_raw_pts > 0:
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
        self._zero_pt_count: int = 0     # consecutive frames with zero valid forward points
        self.lidar_degraded: bool = False # True = rain/blocked (data arriving but no points)
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


LEVEL_RANK = {"SAFE": 0, "CAUTION": 1, "IMMINENT": 2}


def lidar_raw_level(fl, fc, fr) -> str:
    """Raw LiDAR level — used for display only, NOT for collision decision."""
    min_d = min((d for d in [fl, fc, fr] if d is not None), default=None)
    if min_d is None: return "SAFE"
    if min_d <= RANGE_IMMINENT or min_d <= STOP_DIST_M: return "IMMINENT"
    if min_d <= RANGE_CAUTION  or min_d <= SLOW_DIST_M: return "CAUTION"
    return "SAFE"


def _lidar_level_from_dist(min_d):
    """Translate minimum LiDAR distance to collision level."""
    if min_d is None:                                      return "SAFE"
    if min_d <= RANGE_IMMINENT or min_d <= STOP_DIST_M:   return "IMMINENT"
    if min_d <= RANGE_CAUTION  or min_d <= SLOW_DIST_M:   return "CAUTION"
    return "SAFE"


def lidar_yolo_fused_level(fl, fc, fr, lidar_state: str,
                            cam_dets: list, cam_stale: bool,
                            lidar_stale: bool) -> tuple:
    """
    Graceful-degradation fusion for LiDAR + Camera.

    All-sensors nominal:
      LiDAR hit + YOLO confirms  → full IMMINENT/CAUTION from LiDAR distance
      LiDAR hit + YOLO nothing   → camera blocked/dark — full LiDAR level still fires
      YOLO hit  + no LiDAR hit   → CAUTION only (no range)
      Both clear                 → SAFE

    One sensor missing/blocked:
      LiDAR stale, camera OK     → camera-only → IMMINENT (no range, assume worst case)
      Camera stale/blocked, LiDAR OK → LiDAR-only at full level (IMMINENT if close)
      Both stale                 → SAFE (radar carries the decision)

    Returns (level, confirmed, reason_str)
    """
    min_d     = min((d for d in [fl, fc, fr] if d is not None), default=None)
    lidar_hit = min_d is not None and min_d <= SLOW_DIST_M
    yolo_hit  = len(cam_dets) > 0

    # ── Both sensors missing/degraded ────────────────────────────
    # Do NOT return SAFE here — the system is blind on LiDAR+camera.
    # Return CAUTION so the operator knows sensors are impaired and
    # radar is the only active sensor. Radar may still upgrade to
    # IMMINENT if it detects an approaching object.
    if lidar_stale and cam_stale:
        return "CAUTION", False, "LiDAR+Cam:BOTH STALE — radar only"

    # ── Camera missing — LiDAR works alone at full level ─────────
    if cam_stale:
        lvl = _lidar_level_from_dist(min_d)
        reason = f"LiDAR-only(cam stale) d={min_d:.2f}m" if min_d else "LiDAR-only(cam stale) clear"
        return lvl, lidar_hit, reason

    # ── LiDAR missing — camera is the only visual sensor ────────
    # No range data from LiDAR. Use box-height distance bands to estimate
    # how close the object is. NEAR box → IMMINENT, MID/FAR → CAUTION.
    if lidar_stale:
        if yolo_hit:
            labels = ", ".join(f"{d['label']}({d['dist_band']})" for d in cam_dets)
            lvl = max(
                (d["dist_level"] for d in cam_dets),
                key=lambda l: LEVEL_RANK.get(l, 0)
            )
            # Never go below CAUTION when LiDAR is down — something is visible
            if LEVEL_RANK.get(lvl, 0) < LEVEL_RANK["CAUTION"]:
                lvl = "CAUTION"
            return lvl, True, f"Cam-only(LiDAR down) {labels}"
        return "SAFE", False, "Cam-only(LiDAR down) clear"

    # ── Both sensors available ────────────────────────────────────
    if lidar_hit and yolo_hit:
        # Full confirmation — use LiDAR distance for level
        lvl    = _lidar_level_from_dist(min_d)
        labels = ", ".join(d["label"] for d in cam_dets)
        return lvl, True, f"CONFIRMED {labels} @ {min_d:.2f}m"

    if lidar_hit and not yolo_hit:
        # Camera available but sees nothing — may be blocked, dark, or unclassifiable.
        # LiDAR fires at full level regardless; camera cannot veto a range measurement.
        lvl = _lidar_level_from_dist(min_d)
        return lvl, False, f"LiDAR(cam no-detect) d={min_d:.2f}m"

    if not lidar_hit and yolo_hit:
        # Both sensors active but only camera triggered — no LiDAR range confirmation.
        # Use the worst distance band from the detections themselves.
        labels = ", ".join(f"{d['label']}({d['dist_band']})" for d in cam_dets)
        lvl = max(
            (d["dist_level"] for d in cam_dets),
            key=lambda l: LEVEL_RANK.get(l, 0)
        )
        return lvl, True, f"YOLO-only {labels}"

    return "SAFE", False, "clear"


def fuse(radar_worst: str, radar_tracks: List[RadarTrack],
         lidar_state: str, fl, fc, fr,
         cam_dets: list, cam_level: str,
         radar_stale: bool, lidar_stale: bool, cam_stale: bool,
         lidar_degraded: bool = False, cam_degraded: bool = False,
         car_speed_mps: float = None) -> dict:
    """
    3-sensor graceful-degradation fusion.

    Stale    = physically disconnected — no data arriving at all.
    Degraded = connected but impaired — rain on LiDAR, lens blocked on camera.

    Radar static fallback only activates when LiDAR is DEGRADED (rain/blocked),
    not when it's merely disconnected — a disconnected LiDAR is a wiring fault,
    not a weather event, and shouldn't silently change radar behaviour.

    car_speed_mps: current car speed from motor controller. When zero (stopped),
    radar static fallback is suppressed — a stopped car sees everything as static.
    Pass None when motor control is not yet wired (assumes car may be moving).
    """
    # ── Radar level ───────────────────────────────────────────────
    # Normal:   radar alarms on APPROACHING only (Pi sends static as SAFE)
    # Degraded: LiDAR impaired by rain/block → promote static radar tracks,
    #           BUT only if the car is actually moving — a stopped car has
    #           zero relative velocity to everything, making all returns static.
    car_moving = (car_speed_mps is None) or (abs(car_speed_mps) > 0.05)

    if radar_stale:
        lvl_r = "SAFE"
    elif lidar_degraded and car_moving:
        # LiDAR impaired — radar covers static obstacles only while moving
        lvl_r = radar_worst
        for t in radar_tracks:
            if t.cls == "STATIC":
                if t.range_m <= RANGE_IMMINENT:
                    lvl_r = ["SAFE","CAUTION","IMMINENT"][max(LEVEL_RANK[lvl_r], LEVEL_RANK["IMMINENT"])]
                elif t.range_m <= RANGE_CAUTION:
                    lvl_r = ["SAFE","CAUTION","IMMINENT"][max(LEVEL_RANK[lvl_r], LEVEL_RANK["CAUTION"])]
    else:
        lvl_r = radar_worst

    # ── LiDAR + Camera level ──────────────────────────────────────
    lvl_ly, confirmed, ly_reason = lidar_yolo_fused_level(
        fl, fc, fr, lidar_state,
        cam_dets if (not cam_stale and not cam_degraded) else [],
        cam_stale or cam_degraded, lidar_stale or lidar_degraded)

    # ── Final: worst of radar and LiDAR+cam ───────────────────────
    fused = ["SAFE","CAUTION","IMMINENT"][max(LEVEL_RANK[lvl_r], LEVEL_RANK[lvl_ly])]

    # Best TTC from approaching radar tracks
    best_ttc = None
    for t in radar_tracks:
        if not radar_stale and t.ttc_s is not None:
            best_ttc = t.ttc_s if best_ttc is None else min(best_ttc, t.ttc_s)

    min_lidar = min((d for d in [fl, fc, fr] if d is not None), default=None)

    detail_parts = []
    if not radar_stale:
        detail_parts.append(f"Radar:{lvl_r}")
        if best_ttc: detail_parts.append(f"TTC={best_ttc:.1f}s")
    else:
        detail_parts.append("Radar:STALE")
    detail_parts.append(f"LiDAR+Cam:{ly_reason}")

    all_stale = radar_stale and lidar_stale and cam_stale

    return {
        "level":      fused,
        "ttc_s":      best_ttc,
        "min_lidar":  min_lidar,
        "radar_lvl":  lvl_r,
        "lidar_lvl":  lvl_ly,
        "cam_lvl":    cam_level,
        "ly_confirmed": confirmed,
        "ly_reason":  ly_reason,
        "detail":     "  |  ".join(detail_parts),
        "radar_stale":     radar_stale,
        "lidar_stale":     lidar_stale,
        "cam_stale":       cam_stale,
        "all_stale":       all_stale,
        "lidar_degraded":  lidar_degraded,
        "cam_degraded":    cam_degraded,
    }


# ══════════════════════════════════════════════════════════════
#  CAMERA + YOLO DETECTOR
# ══════════════════════════════════════════════════════════════
class CameraDetector(threading.Thread):
    """
    Runs YOLOv8n on the IMX477 CSI camera.
    Only reports detections whose bounding box centre falls
    within the centre CENTRE_ZONE_FRAC of the frame width.
    """
    def __init__(self):
        super().__init__(daemon=True)
        self._stop_evt   = threading.Event()
        self.detections    = []    # list of dicts: {label, conf, cx, cy, x1,y1,x2,y2}
        self.frame_rgb     = None  # latest frame as RGB numpy array for display
        self.cam_level     = "SAFE"
        self.cam_degraded  = False # True = frames arriving but very dark (blocked/covered)
        self.last_update   = 0.0
        self._lock         = threading.Lock()
        self._model      = None
        self._cap        = None

    def stop(self): self._stop_evt.set()

    def get(self):
        with self._lock:
            return (list(self.detections),
                    self.frame_rgb.copy() if self.frame_rgb is not None else None,
                    self.cam_level,
                    self.last_update,
                    self.cam_degraded)

    def _open_camera(self):
        import cv2
        # Try multiple pipelines in order
        # Arducam B0262 IMX477 Mini — 2-lane MIPI, 3.9mm M12 lens, 80 deg FOV
        pipelines = [
            # Pipeline 1: native 640x480 — widest FOV, least zoom
            ("nvarguscamerasrc sensor-id=0 ! "
             "video/x-raw(memory:NVMM), width=640, height=480, framerate=60/1 ! "
             "nvvidconv ! "
             "video/x-raw, width=640, height=480, format=BGRx ! "
             "videoconvert ! video/x-raw, format=BGR ! "
             "queue leaky=downstream max-size-buffers=1 ! "
             "appsink drop=1 max-buffers=1 sync=false"),
            # Pipeline 2: 1280x720 downscaled
            ("nvarguscamerasrc sensor-id=0 ! "
             "video/x-raw(memory:NVMM), width=1280, height=720, framerate=30/1 ! "
             "nvvidconv ! video/x-raw, width=640, height=480, format=BGRx ! "
             "videoconvert ! video/x-raw, format=BGR ! "
             "appsink drop=1 max-buffers=1 sync=false"),
            # Pipeline 3: let nvargus pick
            ("nvarguscamerasrc sensor-id=0 ! "
             "video/x-raw(memory:NVMM) ! "
             "nvvidconv ! video/x-raw, width=640, height=480, format=BGRx ! "
             "videoconvert ! video/x-raw, format=BGR ! "
             "appsink drop=1 max-buffers=1 sync=false"),
        ]
        for i, pipeline in enumerate(pipelines):
            print(f"[CAM] Trying pipeline {i+1}...")
            try:
                cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
                if cap.isOpened():
                    # Try reading a few frames to confirm it's working
                    for _ in range(5):
                        ret, frame = cap.read()
                        if ret and frame is not None and frame.mean() > 0.5:
                            print(f"[CAM] Pipeline {i+1} working!")
                            return cap
                    cap.release()
            except Exception as e:
                print(f"[CAM] Pipeline {i+1} exception: {e}")
        print("[CAM] All GStreamer pipelines failed — camera disabled")
        return None

    def run(self):
        import cv2
        # Load YOLO on GPU — mock torchvision if not available
        try:
            import sys, types, torch
            device = "cuda" if torch.cuda.is_available() else "cpu"

            # ── Full torchvision mock for Jetson (no compatible binary) ──
            import enum, types as _t

            def _nms(boxes, scores, iou_threshold):
                if boxes.numel()==0: return torch.zeros(0,dtype=torch.long)
                x1,y1,x2,y2=boxes[:,0],boxes[:,1],boxes[:,2],boxes[:,3]
                areas=(x2-x1)*(y2-y1); order=scores.argsort(descending=True); keep=[]
                while order.numel()>0:
                    i=order[0].item(); keep.append(i)
                    if order.numel()==1: break
                    rest=order[1:]
                    inter=((x2[rest].clamp(max=float(x2[i]))-x1[rest].clamp(min=float(x1[i]))).clamp(0)*
                           (y2[rest].clamp(max=float(y2[i]))-y1[rest].clamp(min=float(y1[i]))).clamp(0))
                    order=rest[inter/(areas[i]+areas[rest]-inter+1e-6)<=iou_threshold]
                return torch.tensor(keep,dtype=torch.long)

            class _IM(enum.Enum):
                NEAREST=0; BILINEAR=2; BICUBIC=3; LANCZOS=1; HAMMING=5; BOX=4

            class _Compose:
                def __init__(self,t): self.transforms=t
                def __call__(self,x):
                    for t in self.transforms: x=t(x) if callable(t) else x
                    return x

            def _mk(n): return _t.ModuleType(n)
            _tv=_mk('torchvision'); _tv.__version__='0.20.0'
            _ops=_mk('torchvision.ops')
            _ops.nms=_nms
            _ops.box_iou=lambda a,b:torch.zeros(len(a),len(b))
            _ops.box_area=lambda b:(b[:,2]-b[:,0])*(b[:,3]-b[:,1])
            _ops.batched_nms=lambda b,s,i,t:_nms(b,s,t)
            _ops.clip_boxes_to_image=lambda b,s:b
            _ops.remove_small_boxes=lambda b,m:torch.arange(len(b))
            _ops_b=_mk('torchvision.ops.boxes'); _ops_b.nms=_nms; _ops_b.box_iou=_ops.box_iou
            _tf=_mk('torchvision.transforms')
            _tf.InterpolationMode=_IM; _tf.Compose=_Compose
            for _n in ('ToTensor','Normalize','Resize','CenterCrop','RandomHorizontalFlip',
                       'RandomCrop','ColorJitter','RandomRotation','Grayscale','Pad'):
                setattr(_tf,_n,type(_n,(),{'__init__':lambda s,*a,**k:None,'__call__':lambda s,x:x}))
            _tff=_mk('torchvision.transforms.functional')
            _tff.InterpolationMode=_IM
            _tff.resize=lambda img,size,**k:img
            _tff.normalize=lambda t,m,s,**k:t
            _tff.to_tensor=lambda x:x
            _mod=_mk('torchvision.models')
            _util=_mk('torchvision.utils'); _util.make_grid=lambda *a,**k:None
            _io=_mk('torchvision.io')
            _ds=_mk('torchvision.datasets')
            for _cn in ('ImageFolder','CIFAR10','CIFAR100','ImageNet','MNIST','CocoDetection'):
                setattr(_ds,_cn,type(_cn,(),{'__init__':lambda s,*a,**k:None}))
            _mods={
                'torchvision':_tv,'torchvision.ops':_ops,'torchvision.ops.boxes':_ops_b,
                'torchvision.transforms':_tf,'torchvision.transforms.functional':_tff,
                'torchvision.models':_mod,'torchvision.utils':_util,
                'torchvision.io':_io,'torchvision.datasets':_ds,
            }
            for _k,_v in _mods.items(): sys.modules[_k]=_v
            _tv.ops=_ops; _tv.transforms=_tf; _tv.models=_mod
            _tv.utils=_util; _tv.io=_io; _tv.datasets=_ds
            print("[YOLO] torchvision fully mocked for Jetson")

            from ultralytics import YOLO
            self._model = YOLO(YOLO_MODEL)
            self._model.to(device)
            self._device = device
            print(f"[YOLO] Model loaded: {YOLO_MODEL} on {device.upper()}")
        except Exception as e:
            print(f"[YOLO] Failed to load model: {e}")
            print("       pip install ultralytics")
            return

        # Open camera
        self._cap = self._open_camera()
        if self._cap is None:
            print("[CAM] Cannot open camera — camera detection disabled")
            return

        frame_interval = 1.0 / CAM_FPS
        last_inf  = 0.0
        frame_ctr = 0

        while not self._stop_evt.is_set():
            ret, frame = self._cap.read()
            if not ret:
                time.sleep(0.05); continue

            frame_ctr += 1
            now = time.time()

            h, w = frame.shape[:2]
            cx_min = w * (0.5 - CENTRE_ZONE_FRAC / 2)
            cx_max = w * (0.5 + CENTRE_ZONE_FRAC / 2)

            # Always draw zone lines on every frame for smooth display.
            # Bounding boxes from the last YOLO inference are re-drawn each frame
            # so they stay visible between inference frames.
            with self._lock:
                last_dets = list(self.detections)

            display = frame.copy()
            cv2.line(display, (int(cx_min),0), (int(cx_min),h), (0,200,200), 1)
            cv2.line(display, (int(cx_max),0), (int(cx_max),h), (0,200,200), 1)
            for d in last_dets:
                if d.get("foreign"):
                    col = (0,165,255)
                elif d["dist_band"] == "NEAR":
                    col = (0,0,255)
                elif d["dist_band"] == "MID":
                    col = (0,140,255)
                else:
                    col = (0,255,0)
                cv2.rectangle(display,
                              (int(d["x1"]),int(d["y1"])),
                              (int(d["x2"]),int(d["y2"])), col, 2)
                cv2.putText(display,
                            f"{d['label']} {d['dist_band']} {d['conf']:.2f}",
                            (int(d["x1"]), int(d["y1"])-6),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 1)

            rgb = cv2.cvtColor(display, cv2.COLOR_BGR2RGB)

            # Degraded detection — check mean brightness of raw frame (not annotated).
            # A blocked/covered lens delivers very dark frames even though the camera
            # is physically connected and streaming. Rain has a similar effect.
            mean_brightness = float(frame.mean())
            cam_degraded = mean_brightness < CAM_DARK_THRESHOLD

            with self._lock:
                self.frame_rgb    = rgb
                self.cam_degraded = cam_degraded
                # Clear cached detections when degraded — stale detections from
                # before the lens was blocked must not continue triggering alarms
                if cam_degraded:
                    self.detections = []
                    self.cam_level  = "SAFE"
                self._last_brightness = mean_brightness

            # Only run YOLO every Nth frame and respecting rate limit
            if frame_ctr % YOLO_EVERY_N != 0:
                continue
            if now - last_inf < frame_interval:
                continue
            last_inf = now

            try:
                results = self._model(frame, conf=YOLO_CONF,
                                      imgsz=YOLO_IMG_SIZE,
                                      device=self._device, verbose=False)[0]
            except Exception as e:
                print(f"[YOLO] Inference error: {e}"); continue

            dets = []
            for box in results.boxes:
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                cx = (x1 + x2) / 2
                cy = (y1 + y2) / 2
                # Only detections in centre zone
                if not (cx_min <= cx <= cx_max):
                    continue
                conf  = float(box.conf[0])
                cls   = int(box.cls[0])
                # Resolve class name — anything not in KNOWN_CLASSES = "Foreign Object"
                # This ensures unrecognised objects are still detected and flagged
                known = results.names
                if isinstance(known, dict):
                    raw_label = known.get(cls, "Foreign Object")
                elif cls < len(known):
                    raw_label = known[cls]
                else:
                    raw_label = "Foreign Object"
                label = raw_label if raw_label in KNOWN_CLASSES else "Foreign Object"
                is_foreign = (label == "Foreign Object")
                box_h  = y2 - y1
                d_band = cam_dist_band(box_h)
                d_lvl  = cam_dist_level(box_h)
                dets.append({"label":label, "conf":conf,
                             "cx":cx, "cy":cy,
                             "x1":x1,"y1":y1,"x2":x2,"y2":y2,
                             "foreign":is_foreign,
                             "box_h":box_h,
                             "dist_band":d_band,
                             "dist_level":d_lvl})

            # Camera collision level — derived from worst distance band across all detections.
            # Box height → distance band → collision level (no calibration needed).
            if len(dets) == 0:
                cam_level = "SAFE"
            else:
                cam_level = max(
                    (d["dist_level"] for d in dets),
                    key=lambda l: LEVEL_RANK.get(l, 0)
                )

            with self._lock:
                self.detections  = dets
                self.cam_level   = cam_level
                self.last_update = now

        if self._cap: self._cap.release()


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
    def __init__(self, lidar: LidarReader, radar: RadarReceiver, camera: CameraDetector):
        self.lidar  = lidar
        self.radar  = radar
        self.camera = camera
        self._fc = 0; self._fps = 0.0; self._ft = time.time()
        self._radar_trails: dict = {}
        self._radar_labels: dict = {}

        # Motor controller
        try:
            from motor_controller import MotorController
            self.motor = MotorController()
        except Exception as e:
            print(f"[MOTOR] Not loaded: {e}")
            self.motor = None

        # Data logger
        try:
            from data_logger import DataLogger
            self.logger = DataLogger()
        except Exception as e:
            print(f"[LOGGER] Not loaded: {e}")
            self.logger = None

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
        self.lbl_camera = self._make_status_lbl("CAM    ·  waiting...")
        self.lbl_fused  = self._make_status_lbl("FUSED  ·  —")
        for lbl in [self.lbl_radar, self.lbl_lidar, self.lbl_camera, self.lbl_fused]:
            status_row.addWidget(lbl)

        # Sensor health warning — shown prominently when sensors go offline
        self.lbl_health = QtWidgets.QLabel("✔  ALL SENSORS ONLINE")
        self.lbl_health.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self.lbl_health.setFont(QtGui.QFont("Courier New", 11, QtGui.QFont.Weight.Bold))
        self.lbl_health.setFixedHeight(32)
        self.lbl_health.setStyleSheet(f"background:transparent;color:{GRN};"
                                      f"border:1px solid {GRN};border-radius:4px;padding:3px;")
        root.addWidget(self.lbl_health)

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

        # CAMERA panel
        cam_container = QtWidgets.QWidget()
        cam_container.setStyleSheet(f"background:{BG};")
        cam_vbox = QtWidgets.QVBoxLayout(cam_container)
        cam_vbox.setContentsMargins(0,0,0,0); cam_vbox.setSpacing(3)
        cam_title = QtWidgets.QLabel("CAMERA  ·  YOLOv8n  ·  Centre Zone")
        cam_title.setFont(QtGui.QFont("Courier New",9,QtGui.QFont.Weight.Bold))
        cam_title.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        cam_title.setStyleSheet(f"color:{AMBER};")
        cam_vbox.addWidget(cam_title)
        self.cam_label = QtWidgets.QLabel()
        self.cam_label.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self.cam_label.setMinimumSize(640, 480)
        self.cam_label.setStyleSheet(f"background:#000;border:1px solid {DIMGRN};")
        cam_vbox.addWidget(self.cam_label, stretch=1)
        self.cam_det_lbl = QtWidgets.QLabel("No detections")
        self.cam_det_lbl.setFont(QtGui.QFont("Courier New",8))
        self.cam_det_lbl.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self.cam_det_lbl.setStyleSheet(f"color:{GRN};")
        cam_vbox.addWidget(self.cam_det_lbl)
        panels.addWidget(cam_container, stretch=4)

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
        pts, fl, fc, fr, lidar_state, lidar_t, lidar_degraded = self.lidar.get()
        radar_tracks, radar_worst, radar_t                     = self.radar.get()
        cam_dets, cam_frame, cam_level, cam_t, cam_degraded    = self.camera.get()

        # Stale = physically disconnected (no data at all)
        radar_stale = (now - radar_t) > STALE_S
        lidar_stale = (now - lidar_t) > STALE_S
        cam_stale   = (now - cam_t)   > STALE_S * 4  # camera runs slower
        # Degraded = connected but impaired (rain, blocked lens, dirty optics)
        # These are independent of stale — a connected sensor can be degraded

        # ── Fuse ──────────────────────────────────────────────────
        fusion = fuse(radar_worst, radar_tracks,
                      lidar_state, fl, fc, fr,
                      cam_dets, cam_level,
                      radar_stale, lidar_stale, cam_stale,
                      lidar_degraded=lidar_degraded,
                      cam_degraded=cam_degraded)
        level = fusion["level"]

        # Motor control
        if self.motor:
            self.motor.update(level, fl, fc, fr)

        # Data logging — only saves when obstacle detected
        if self.logger:
            motor_state = self.motor._state if self.motor else "UNKNOWN"
            self.logger.log(level, fl, fc, fr,
                            radar_tracks, cam_dets, motor_state)

        # ── Banner ────────────────────────────────────────────────
        self._set_banner(level)

        # ── Sensor status labels ───────────────────────────────────
        n_dets  = len(cam_dets)
        ttc_str = (f"TTC:{fusion['ttc_s']:.1f}s" if fusion["ttc_s"] else "TTC:—")

        if radar_stale:
            self._set_status(self.lbl_radar, "RADAR  ·  ✖ OFFLINE", ok=False)
        else:
            self._set_status(self.lbl_radar,
                f"RADAR  ·  {radar_worst}  ·  {len(radar_tracks)} tracks", ok=True)

        if lidar_stale:
            self._set_status(self.lbl_lidar, "LiDAR  ·  ✖ DISCONNECTED", ok=False)
        elif fusion.get("lidar_degraded"):
            self._set_status(self.lbl_lidar,
                f"LiDAR  ·  ⚠ IMPAIRED — no forward returns ({self.lidar._zero_pt_count}f)"
                f"  ·  radar static fallback ACTIVE", ok=False)
        else:
            self._set_status(self.lbl_lidar,
                f"LiDAR  ·  {lidar_state}" +
                (f"  ·  {fusion['min_lidar']:.2f}m" if fusion["min_lidar"] else "") +
                ("  ✓" if fusion.get("ly_confirmed") else ""), ok=True)

        if cam_stale:
            self._set_status(self.lbl_camera,
                "CAM  ·  ✖ DISCONNECTED" + ("  [NO SIGNAL]" if cam_t == 0 else ""),
                ok=False)
        elif fusion.get("cam_degraded"):
            brightness = getattr(self.camera, '_last_brightness', 0)
            self._set_status(self.lbl_camera,
                f"CAM  ·  ⚠ IMPAIRED — lens blocked  ·  brightness={brightness:.0f}", ok=False)
        else:
            brightness = getattr(self.camera, '_last_brightness', 0)
            self._set_status(self.lbl_camera,
                f"CAM  ·  {fusion['cam_lvl']}  ·  {n_dets} obj  ·  bri={brightness:.0f}", ok=True)

        lvl_col = RED if level=="IMMINENT" else (AMBER if level=="CAUTION" else GRN)
        self.lbl_fused.setText(f"FUSED  ·  {level}  ·  {ttc_str}")
        self.lbl_fused.setStyleSheet(
            f"background:#0a1f0a;color:{lvl_col};"
            f"border:1px solid {lvl_col};border-radius:3px;padding:2px;")

        # ── Sensor health warning bar ──────────────────────────────
        # Separate disconnected (stale) from impaired (degraded — rain/blocked)
        offline  = []
        impaired = []
        if radar_stale:                          offline.append("RADAR")
        if lidar_stale:                          offline.append("LiDAR")
        elif fusion.get("lidar_degraded"):       impaired.append("LiDAR")
        if cam_stale:                            offline.append("CAM")
        elif fusion.get("cam_degraded"):         impaired.append("CAM")

        if fusion.get("all_stale"):
            self.lbl_health.setText("⚠  ALL SENSORS OFFLINE — SYSTEM BLIND — NO COLLISION DETECTION  ⚠")
            self.lbl_health.setStyleSheet(
                f"background:{RED};color:#fff;font-weight:bold;"
                f"border:2px solid #ff0000;border-radius:4px;padding:3px;")
        elif len(offline) == 2:
            if "RADAR" not in offline:
                msg = f"⚠  {' + '.join(offline)} DISCONNECTED — RADAR ONLY — STATIC OBSTACLES MAY BE MISSED  ⚠"
            elif "LiDAR" not in offline:
                msg = f"⚠  {' + '.join(offline)} DISCONNECTED — LiDAR ONLY — NO VELOCITY DATA  ⚠"
            else:
                msg = f"⚠  {' + '.join(offline)} DISCONNECTED — CAMERA ONLY — NO RANGE DATA  ⚠"
            self.lbl_health.setText(msg)
            self.lbl_health.setStyleSheet(
                f"background:{RED};color:#fff;font-weight:bold;"
                f"border:2px solid {RED};border-radius:4px;padding:3px;")
        elif len(offline) == 1:
            capability = {"RADAR": "no velocity/TTC", "LiDAR": "no range", "CAM": "no visual confirm"}
            self.lbl_health.setText(f"⚠  {offline[0]} DISCONNECTED — {capability[offline[0]]}")
            self.lbl_health.setStyleSheet(
                f"background:#1a1a00;color:{AMBER};font-weight:bold;"
                f"border:1px solid {AMBER};border-radius:4px;padding:3px;")
        elif impaired:
            # Degraded — connected but impaired by weather/obstruction
            msgs = []
            if "LiDAR" in impaired: msgs.append("LiDAR IMPAIRED — rain or blocked lens — radar static fallback ACTIVE")
            if "CAM"   in impaired: msgs.append("CAM IMPAIRED — lens blocked or very dark")
            self.lbl_health.setText("⚠  " + "  |  ".join(msgs))
            self.lbl_health.setStyleSheet(
                f"background:#1a0a00;color:{AMBER};font-weight:bold;"
                f"border:1px solid {AMBER};border-radius:4px;padding:3px;")
        else:
            self.lbl_health.setText("✔  ALL SENSORS ONLINE")
            self.lbl_health.setStyleSheet(
                f"background:transparent;color:{GRN};"
                f"border:1px solid {GRN};border-radius:4px;padding:3px;")

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

        # ── Camera frame display ──────────────────────────────────
        if cam_frame is not None:
            try:
                import cv2
                disp = cv2.resize(cam_frame, (640, 480))
                qimg = QtGui.QImage(disp.data, 640, 480,
                                    640*3, QtGui.QImage.Format.Format_RGB888)
                self.cam_label.setPixmap(QtGui.QPixmap.fromImage(qimg))
                if cam_dets:
                    det_str = "  ".join(
                        f"{d['label']}({d['conf']:.2f})" for d in cam_dets)
                    col = RED if fusion["cam_lvl"]=="IMMINENT" else (
                          AMBER if fusion["cam_lvl"]=="CAUTION" else GRN)
                    self.cam_det_lbl.setText(det_str)
                    self.cam_det_lbl.setStyleSheet(f"color:{col};")
                else:
                    self.cam_det_lbl.setText("No objects in centre zone")
                    self.cam_det_lbl.setStyleSheet(f"color:{GRN};")
            except Exception:
                pass
        else:
            self.cam_label.setText("Camera initialising...")
            self.cam_label.setStyleSheet(
                f"background:#000;color:{GREY};border:1px solid {DIMGRN};")

        self.sbar.setText(
            f"  FPS:{self._fps:.1f}  |  "
            f"Radar:{len(radar_tracks)}trk  |  "
            f"LiDAR:{len(pts)}pts  |  "
            f"Cam:{n_dets}obj  |  "
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

    lidar  = LidarReader()
    radar  = RadarReceiver()
    camera = CameraDetector()
    lidar.start()
    radar.start()
    camera.start()

    print("[INFO] All sensors started. Launching GUI...")
    try:
        FusionGUI(lidar, radar, camera).run()
    except KeyboardInterrupt:
        print("\n[EXIT]")
    finally:
        lidar.stop(); radar.stop(); camera.stop()

if __name__ == "__main__":
    main()