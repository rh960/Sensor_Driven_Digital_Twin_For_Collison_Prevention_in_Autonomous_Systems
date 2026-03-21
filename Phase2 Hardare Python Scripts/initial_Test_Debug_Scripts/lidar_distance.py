"""
LD06 Corridor Detector — Fixed + GUI
=====================================
Bugs fixed vs original:
  1. buf trimming (buf[-4096:]) happened BEFORE parsing → data loss on busy streams.
     Fix: trim only AFTER the parse loop, and only on leftover bytes.
  2. DEBUG_POINTS printed every loop tick until a point was found → console spam.
     Fix: rate-limited with DEBUG_PRINT_INTERVAL timestamp gate.
  3. overall_state parameter shadowed Python built-in 'C' (minor but confusing).
     Fix: renamed params to fl/fc/fr throughout.
  4. No manoeuvre hint existed. Added manoeuvre_hint() that suggests a turn direction.

New GUI features:
  • Real-time top-down scatter plot of point cloud coloured by zone
  • Corridor + center-strip overlays, STOP/SLOW distance lines
  • Big status badge: CLEAR / SLOW / STOP / NO DATA / MANOEUVRE
  • FL / FC / FR distances live readout
  • ⏹ STOP  ▶ RESUME  ↩ MANOEUVRE  ✕ QUIT buttons
  • Fallback to headless console loop if matplotlib/tkinter unavailable
"""

import time
import threading
from collections import deque

import numpy as np
import serial

# ── GUI imports (optional) ────────────────────────────────────────────────
try:
    import tkinter as tk
    from tkinter import font as tkfont
    import matplotlib
    matplotlib.use("TkAgg")
    import matplotlib.pyplot as plt
    from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
    from matplotlib.patches import Rectangle
    GUI_AVAILABLE = True
except ImportError:
    GUI_AVAILABLE = False
    print("[WARN] matplotlib/tkinter not available — running headless")

# ── LD06 packet layout ────────────────────────────────────────────────────
PKT_LEN       = 47
HDR0          = 0x54
HDR1          = 0x2C
PTS_PER_PKT   = 12

# ── Serial ────────────────────────────────────────────────────────────────
PORT = "/dev/ttyTHS1"
BAUD = 230400

# ── Mounting ──────────────────────────────────────────────────────────────
ANGLE_OFFSET_DEG = 0.0
Y_AXIS_SIGN      = +1.0   # set -1.0 if left/right swapped

# ── Corridor geometry (m) ─────────────────────────────────────────────────
LOOKAHEAD_M         = 3.0
HALF_WIDTH_M        = 0.70
CENTER_HALF_WIDTH_M = 0.22

# ── Thresholds ────────────────────────────────────────────────────────────
STOP_DIST_M = 0.80
SLOW_DIST_M = 1.80
MIN_VALID_M = 0.08
MAX_VALID_M = 12.0

# ── Stability ─────────────────────────────────────────────────────────────
WINDOW_SEC           = 0.30
PRINT_HZ             = 10
DEBUG_POINTS         = True
DEBUG_PRINT_INTERVAL = 2.0   # seconds between debug console prints


# ─────────────────────────────────────────────────────────────────────────
#  Packet decoder
# ─────────────────────────────────────────────────────────────────────────
def decode_packet(pkt: bytes):
    """
    LD06 47-byte packet:
      [0-1]   0x54 0x2C  headers
      [2-3]   speed
      [4-5]   start_angle / 100 → degrees
      [6-41]  12 × (dist_l, dist_h, confidence)
      [42-43] end_angle / 100 → degrees
      [44-45] timestamp
      [46]    CRC
    Returns list of (angle_deg, dist_m) for valid points, or None.
    """
    if len(pkt) != PKT_LEN:
        return None
    if pkt[0] != HDR0 or pkt[1] != HDR1:
        return None

    start_angle = (pkt[4] | (pkt[5] << 8)) / 100.0
    end_angle   = (pkt[42] | (pkt[43] << 8)) / 100.0

    base = 6
    d_mm = []
    for i in range(PTS_PER_PKT):
        dist = pkt[base + 3 * i] | (pkt[base + 3 * i + 1] << 8)
        d_mm.append(dist)

    ea = end_angle
    sa = start_angle
    if ea < sa:
        ea += 360.0

    angles = np.linspace(sa, ea, PTS_PER_PKT, endpoint=False)
    angles = (angles + ANGLE_OFFSET_DEG) % 360.0

    d_m   = np.array(d_mm, dtype=np.float32) / 1000.0
    valid = (d_m >= MIN_VALID_M) & (d_m <= MAX_VALID_M)

    out = [(float(a), float(d)) for a, d, ok in zip(angles, d_m, valid) if ok]
    return out if out else None


def polar_to_xy(angle_deg, dist_m):
    """Car frame: x forward, y left."""
    th = np.deg2rad(angle_deg)
    x  = dist_m * np.cos(th)
    y  = dist_m * np.sin(th) * Y_AXIS_SIGN
    return x, y


def overall_state(fl, fc, fr):
    for d in (fc, fl, fr):
        if d is not None and d <= STOP_DIST_M:
            return "STOP"
    for d in (fc, fl, fr):
        if d is not None and d <= SLOW_DIST_M:
            return "SLOW"
    if fl is None and fc is None and fr is None:
        return "NO DATA"
    return "CLEAR"


def manoeuvre_hint(fl, fc, fr):
    """Suggest direction with most free space when blocked ahead."""
    if fc is not None and fc <= STOP_DIST_M:
        lv = fl if fl is not None else LOOKAHEAD_M
        rv = fr if fr is not None else LOOKAHEAD_M
        if lv > rv + 0.05:
            return "← TURN LEFT"
        elif rv > lv + 0.05:
            return "TURN RIGHT →"
        else:
            return "↩ REVERSE"
    return ""


# ─────────────────────────────────────────────────────────────────────────
#  Detector thread
# ─────────────────────────────────────────────────────────────────────────
class CorridorDetector(threading.Thread):
    def __init__(self):
        super().__init__(daemon=True)
        self.stop_flag = threading.Event()
        self._lock     = threading.Lock()

        self.front_left     = None
        self.front_center   = None
        self.front_right    = None
        self.last_update_ts = 0.0
        self._points: list  = []   # [(x, y, zone), ...] for GUI

    def stop(self):
        self.stop_flag.set()

    def get(self):
        with self._lock:
            return (self.front_left, self.front_center,
                    self.front_right, self.last_update_ts,
                    list(self._points))

    def run(self):
        _debug_ts = 0.0

        while not self.stop_flag.is_set():
            try:
                ser = serial.Serial(PORT, BAUD, timeout=0.2)
                ser.reset_input_buffer()
                buf = bytearray()
                pts_hist: deque = deque()

                while not self.stop_flag.is_set():
                    chunk = ser.read(1024)
                    if chunk:
                        buf.extend(chunk)

                    # ── BUG-FIX 1: parse first, THEN trim ──────────────
                    i = 0
                    while i + PKT_LEN <= len(buf):
                        if buf[i] == HDR0 and buf[i + 1] == HDR1:
                            pkt = bytes(buf[i: i + PKT_LEN])
                            pts = decode_packet(pkt)
                            if pts:
                                t = time.time()
                                for ang, d in pts:
                                    x, y = polar_to_xy(ang, d)
                                    pts_hist.append((t, x, y))
                            i += PKT_LEN
                        else:
                            i += 1

                    del buf[:i]                   # remove consumed bytes
                    if len(buf) > 4096:           # cap remaining buffer
                        buf = buf[-4096:]
                    # ────────────────────────────────────────────────────

                    now    = time.time()
                    cutoff = now - WINDOW_SEC
                    while pts_hist and pts_hist[0][0] < cutoff:
                        pts_hist.popleft()

                    L_min = None
                    C_min = None
                    R_min = None
                    snap  = []

                    for _, x, y in pts_hist:
                        in_fwd = 0.0 < x <= LOOKAHEAD_M
                        in_lat = abs(y) <= HALF_WIDTH_M

                        if not in_fwd:
                            continue

                        if not in_lat:
                            snap.append((x, y, "out"))
                            continue

                        d_fwd = x
                        if abs(y) <= CENTER_HALF_WIDTH_M:
                            zone = "center"
                            if C_min is None or d_fwd < C_min:
                                C_min = d_fwd
                        elif y > 0:
                            zone = "left"
                            if L_min is None or d_fwd < L_min:
                                L_min = d_fwd
                        else:
                            zone = "right"
                            if R_min is None or d_fwd < R_min:
                                R_min = d_fwd
                        snap.append((x, y, zone))

                    # ── BUG-FIX 2: rate-limited debug print ─────────────
                    if DEBUG_POINTS and (now - _debug_ts) > DEBUG_PRINT_INTERVAL:
                        sample = [(x, y) for (_, x, y) in pts_hist
                                  if 0.2 < x < 2.0 and abs(y) < 1.0]
                        if sample:
                            sx, sy = sample[0]
                            print(f"[DEBUG] x={sx:.2f}m y={sy:.2f}m "
                                  f"(y>0=LEFT) pts={len(pts_hist)}")
                            _debug_ts = now

                    with self._lock:
                        self.front_left     = L_min
                        self.front_center   = C_min
                        self.front_right    = R_min
                        self.last_update_ts = now
                        self._points        = snap

                ser.close()

            except Exception as e:
                print(f"[ERROR] {e} — reconnecting in 1s")
                time.sleep(1.0)


# ─────────────────────────────────────────────────────────────────────────
#  GUI
# ─────────────────────────────────────────────────────────────────────────
STATE_COLORS = {
    "CLEAR":      "#00e676",
    "SLOW":       "#ffea00",
    "STOP":       "#ff1744",
    "NO DATA":    "#546e7a",
    "MANOEUVRE":  "#ff9100",
}
STATE_FG = {
    "CLEAR":      "#0d1117",
    "SLOW":       "#0d1117",
    "STOP":       "#ffffff",
    "NO DATA":    "#ffffff",
    "MANOEUVRE":  "#ffffff",
}

ZONE_COLORS = {
    "left":   "#79c0ff",
    "center": "#ff7b72",
    "right":  "#79c0ff",
    "out":    "#3d444d",
}


class CorridorGUI:
    def __init__(self, detector: CorridorDetector):
        self.det      = detector
        self.running  = True
        self._override = None   # None | "STOP" | "MANOEUVRE"

        # ── Root window ─────────────────────────────────────────────────
        self.root = tk.Tk()
        self.root.title("LD06 Corridor Monitor")
        self.root.configure(bg="#0d1117")
        self.root.geometry("820x680")

        BIG  = ("Courier New", 26, "bold")
        MONO = ("Courier New", 12, "bold")
        SM   = ("Courier New",  9)

        # ── Status bar ───────────────────────────────────────────────────
        top = tk.Frame(self.root, bg="#0d1117")
        top.pack(fill=tk.X, padx=12, pady=(10, 0))

        self.lbl_state = tk.Label(
            top, text="NO DATA", font=BIG,
            bg="#546e7a", fg="#ffffff",
            width=14, anchor="center",
            padx=10, pady=8, relief="flat"
        )
        self.lbl_state.pack(side=tk.LEFT)

        self.lbl_hint = tk.Label(
            top, text="", font=MONO,
            bg="#0d1117", fg="#ff9100", anchor="w"
        )
        self.lbl_hint.pack(side=tk.LEFT, padx=18)

        self.lbl_dist = tk.Label(
            top, text="FL=---  FC=---  FR=---", font=MONO,
            bg="#0d1117", fg="#cdd6f4", anchor="e"
        )
        self.lbl_dist.pack(side=tk.RIGHT, padx=10)

        # ── Plot ─────────────────────────────────────────────────────────
        self.fig = plt.Figure(figsize=(8, 5.4), facecolor="#0d1117")
        self.ax  = self.fig.add_subplot(111)
        self._style_axes()

        self.canvas = FigureCanvasTkAgg(self.fig, master=self.root)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True,
                                         padx=12, pady=6)

        # ── Button row ────────────────────────────────────────────────────
        btn_row = tk.Frame(self.root, bg="#0d1117")
        btn_row.pack(fill=tk.X, padx=12, pady=(0, 12))

        BTN = dict(font=MONO, relief="flat", padx=16, pady=7,
                   cursor="hand2", bd=0, activeforeground="#ffffff")

        tk.Button(btn_row, text="⏹  STOP",
                  bg="#ff1744", fg="white",
                  activebackground="#c62828",
                  command=self._cmd_stop, **BTN
                  ).pack(side=tk.LEFT, padx=4)

        tk.Button(btn_row, text="▶  RESUME",
                  bg="#00c853", fg="#0d1117",
                  activebackground="#00e676",
                  command=self._cmd_resume, **BTN
                  ).pack(side=tk.LEFT, padx=4)

        tk.Button(btn_row, text="↩  MANOEUVRE",
                  bg="#e65100", fg="white",
                  activebackground="#ff9100",
                  command=self._cmd_manoeuvre, **BTN
                  ).pack(side=tk.LEFT, padx=4)

        tk.Button(btn_row, text="✕  QUIT",
                  bg="#21262d", fg="#cdd6f4",
                  activebackground="#30363d",
                  command=self._cmd_quit, **BTN
                  ).pack(side=tk.RIGHT, padx=4)

        self.root.protocol("WM_DELETE_WINDOW", self._cmd_quit)
        self._schedule_update()

    # ── Button callbacks ──────────────────────────────────────────────────
    def _cmd_stop(self):       self._override = "STOP"
    def _cmd_resume(self):     self._override = None
    def _cmd_manoeuvre(self):  self._override = "MANOEUVRE"
    def _cmd_quit(self):
        self.running = False
        self.det.stop()
        self.root.quit()

    # ── Axes styling ──────────────────────────────────────────────────────
    def _style_axes(self):
        ax = self.ax
        ax.set_facecolor("#161b22")
        ax.set_xlim(-0.3, LOOKAHEAD_M + 0.2)
        ax.set_ylim(-HALF_WIDTH_M - 0.25, HALF_WIDTH_M + 0.25)
        ax.set_xlabel("Forward  x  (m)", color="#8b949e", fontsize=9)
        ax.set_ylabel("Lateral  y  (m)   [+LEFT]", color="#8b949e", fontsize=9)
        ax.tick_params(colors="#8b949e", labelsize=8)
        for sp in ax.spines.values():
            sp.set_edgecolor("#30363d")
        ax.grid(color="#21262d", linewidth=0.5, zorder=0)

        # Corridor
        ax.add_patch(Rectangle(
            (0, -HALF_WIDTH_M), LOOKAHEAD_M, 2 * HALF_WIDTH_M,
            linewidth=1.5, edgecolor="#58a6ff",
            facecolor="#58a6ff0d", zorder=1
        ))
        # Center strip
        ax.add_patch(Rectangle(
            (0, -CENTER_HALF_WIDTH_M),
            LOOKAHEAD_M, 2 * CENTER_HALF_WIDTH_M,
            linewidth=1, edgecolor="#e3b341",
            facecolor="#e3b34112", linestyle="--", zorder=1
        ))
        # STOP / SLOW lines
        ax.axvline(STOP_DIST_M, color="#ff1744", lw=1.3, ls=":",
                   label=f"STOP {STOP_DIST_M}m", zorder=2)
        ax.axvline(SLOW_DIST_M, color="#ffea00", lw=1.0, ls=":",
                   label=f"SLOW {SLOW_DIST_M}m", zorder=2)
        # Robot marker
        ax.plot(0, 0, marker="^", ms=13, color="#79c0ff",
                markeredgecolor="#0d1117", zorder=10)

        ax.legend(loc="upper right", fontsize=7,
                  facecolor="#161b22", edgecolor="#30363d",
                  labelcolor="#8b949e")

        ax.text(LOOKAHEAD_M * 0.65,  HALF_WIDTH_M * 0.65,
                "LEFT",   color="#58a6ff", fontsize=8, alpha=0.45,
                ha="center", va="center", zorder=3)
        ax.text(LOOKAHEAD_M * 0.65,  0,
                "CENTER", color="#e3b341", fontsize=8, alpha=0.45,
                ha="center", va="center", zorder=3)
        ax.text(LOOKAHEAD_M * 0.65, -HALF_WIDTH_M * 0.65,
                "RIGHT",  color="#58a6ff", fontsize=8, alpha=0.45,
                ha="center", va="center", zorder=3)

    # ── Update loop ───────────────────────────────────────────────────────
    def _schedule_update(self):
        if self.running:
            self._do_update()
            self.root.after(int(1000 / PRINT_HZ), self._schedule_update)

    def _do_update(self):
        fl, fc, fr, ts, points = self.det.get()
        stale = (time.time() - ts) > 1.0

        state = "NO DATA" if stale else overall_state(fl, fc, fr)
        hint  = "" if stale else manoeuvre_hint(fl, fc, fr)

        if self._override == "STOP":
            state, hint = "STOP", "⚠ MANUAL STOP"
        elif self._override == "MANOEUVRE":
            state = "MANOEUVRE"
            hint  = hint or "↩ MANOEUVRING"

        # Status badge
        bg = STATE_COLORS.get(state, "#546e7a")
        fg = STATE_FG.get(state, "#ffffff")
        self.lbl_state.config(text=state, bg=bg, fg=fg)
        self.lbl_hint.config(text=hint)

        def fmt(v):
            return "---" if v is None else f"{v:.2f}m"
        self.lbl_dist.config(text=f"FL={fmt(fl)}  FC={fmt(fc)}  FR={fmt(fr)}")

        # ── Redraw scatter ───────────────────────────────────────────────
        self.ax.cla()
        self._style_axes()

        if points:
            xs = np.array([p[0] for p in points])
            ys = np.array([p[1] for p in points])
            cs = [ZONE_COLORS.get(p[2], "#484f58") for p in points]
            self.ax.scatter(xs, ys, c=cs, s=14, alpha=0.80,
                            linewidths=0, zorder=5)

        self.canvas.draw_idle()

    def run(self):
        self.root.mainloop()


# ─────────────────────────────────────────────────────────────────────────
#  Headless console fallback
# ─────────────────────────────────────────────────────────────────────────
def headless_loop(det: CorridorDetector):
    print(f"\nLD06 Corridor Detector  PORT={PORT}  BAUD={BAUD}")
    print(f"LOOKAHEAD={LOOKAHEAD_M}m  HALF_WIDTH={HALF_WIDTH_M}m"
          f"  CENTER={CENTER_HALF_WIDTH_M}m")
    print(f"STOP<{STOP_DIST_M}m  SLOW<{SLOW_DIST_M}m\n")

    try:
        while True:
            fl, fc, fr, ts, _ = det.get()
            stale = (time.time() - ts) > 1.0

            def fmt(v):
                return "---" if v is None else f"{v:.2f}"

            if stale:
                print("[NO DATA]  FL=---  FC=---  FR=---")
            else:
                st   = overall_state(fl, fc, fr)
                hint = manoeuvre_hint(fl, fc, fr)
                line = f"[{st:8s}]  FL={fmt(fl)}m  FC={fmt(fc)}m  FR={fmt(fr)}m"
                if hint:
                    line += f"  {hint}"
                print(line)

            time.sleep(1.0 / PRINT_HZ)

    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        det.stop()


# ─────────────────────────────────────────────────────────────────────────
#  Main
# ─────────────────────────────────────────────────────────────────────────
def main():
    det = CorridorDetector()
    det.start()

    if GUI_AVAILABLE:
        gui = CorridorGUI(det)
        gui.run()
    else:
        headless_loop(det)


if __name__ == "__main__":
    main()