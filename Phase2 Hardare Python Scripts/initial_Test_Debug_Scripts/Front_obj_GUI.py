import time
import threading
from collections import deque

import numpy as np
import serial
import tkinter as tk
from tkinter import ttk

# ---------------- LD06 decode ----------------
PKT_LEN = 47
HDR0 = 0x54
HDR1 = 0x2C
PTS_PER_PKT = 12

# ---------------- Serial ----------------
PORT = "/dev/ttyTHS1"
BAUD = 230400

# ---------------- Front detection tuning ----------------
# Arrow on LD06 top cap = 0° reference. If arrow points forward, keep OFFSET=0.
FRONT_CENTER_DEG = 0.0
FRONT_HALF_CONE = 25.0     # ±25° front cone
ANGLE_OFFSET_DEG = 0.0     # rotate if mounted differently

# Thresholds (edit these)
STOP_DIST_M = 0.80
SLOW_DIST_M = 1.80

MIN_VALID_M = 0.08
MAX_VALID_M = 12.0

# Stability / smoothing
WINDOW_SEC = 0.25          # consider points from last 0.25s
SMOOTH_ALPHA = 0.35        # 0..1 (higher = less smoothing)
UPDATE_HZ = 20             # GUI update rate

# Dashboard scale
DASH_MAX_RANGE_M = 3.0     # what "full scale" means for the bar + graph


def ang_diff_deg(a, b):
    return (a - b + 180.0) % 360.0 - 180.0


def in_front(angle_deg):
    return abs(ang_diff_deg(angle_deg, FRONT_CENTER_DEG)) <= FRONT_HALF_CONE


def decode_packet(pkt: bytes):
    if len(pkt) != PKT_LEN:
        return None
    if pkt[0] != HDR0 or pkt[1] != HDR1:
        return None

    start_angle = (pkt[4] | (pkt[5] << 8)) / 100.0

    d_mm = []
    base = 6
    for i in range(PTS_PER_PKT):
        dist = pkt[base + 3*i] | (pkt[base + 3*i + 1] << 8)
        d_mm.append(dist)

    end_angle = (pkt[42] | (pkt[43] << 8)) / 100.0

    sa = start_angle
    ea = end_angle
    if ea < sa:
        ea += 360.0

    angles = np.linspace(sa, ea, PTS_PER_PKT, endpoint=False)
    angles = (angles + ANGLE_OFFSET_DEG) % 360.0

    d_m = np.array(d_mm, dtype=np.float32) / 1000.0
    valid = (d_m >= MIN_VALID_M) & (d_m <= MAX_VALID_M)

    pts = [(float(a), float(d)) for a, d, ok in zip(angles, d_m, valid) if ok]
    return pts if pts else None


def classify(dist_m):
    if dist_m is None:
        return "NO DATA"
    if dist_m <= STOP_DIST_M:
        return "STOP"
    if dist_m <= SLOW_DIST_M:
        return "SLOW"
    return "CLEAR"


class LD06FrontReader(threading.Thread):
    """
    Reads LD06 packets and continuously updates latest front_min distance.
    """
    def __init__(self, port, baud):
        super().__init__(daemon=True)
        self.port = port
        self.baud = baud
        self.stop_flag = threading.Event()

        self._lock = threading.Lock()
        self.front_min = None
        self.front_min_smoothed = None
        self.last_update_ts = 0.0

    def get(self):
        with self._lock:
            return self.front_min, self.front_min_smoothed, self.last_update_ts

    def run(self):
        while not self.stop_flag.is_set():
            try:
                ser = serial.Serial(self.port, self.baud, timeout=0.2)
                ser.reset_input_buffer()
                buf = bytearray()
                front_hist = deque()  # (t, dist)

                while not self.stop_flag.is_set():
                    chunk = ser.read(1024)
                    if chunk:
                        buf.extend(chunk)

                    if len(buf) > 8192:
                        buf = buf[-4096:]

                    i = 0
                    while i + PKT_LEN <= len(buf):
                        if buf[i] == HDR0 and buf[i + 1] == HDR1:
                            pkt = bytes(buf[i:i + PKT_LEN])
                            pts = decode_packet(pkt)
                            if pts:
                                now = time.time()
                                for ang, dist in pts:
                                    if in_front(ang):
                                        front_hist.append((now, dist))
                            i += PKT_LEN
                        else:
                            i += 1

                    if i > 0:
                        del buf[:i]

                    # drop old hits
                    now = time.time()
                    cutoff = now - WINDOW_SEC
                    while front_hist and front_hist[0][0] < cutoff:
                        front_hist.popleft()

                    # compute min
                    if front_hist:
                        cur_min = float(min(d for _, d in front_hist))
                    else:
                        cur_min = None

                    # smoothing
                    with self._lock:
                        self.front_min = cur_min
                        if cur_min is None:
                            # if no data, keep smoothed as-is but mark update time
                            self.last_update_ts = now
                        else:
                            if self.front_min_smoothed is None:
                                self.front_min_smoothed = cur_min
                            else:
                                a = SMOOTH_ALPHA
                                self.front_min_smoothed = a * cur_min + (1 - a) * self.front_min_smoothed
                            self.last_update_ts = now

                ser.close()

            except Exception as e:
                # brief reconnect loop
                time.sleep(1.0)

    def stop(self):
        self.stop_flag.set()


class DashboardGUI:
    def __init__(self, root):
        self.root = root
        root.title("LD06 Front Collision Dashboard")
        root.geometry("900x520")

        self.reader = LD06FrontReader(PORT, BAUD)
        self.reader.start()

        self.history = deque(maxlen=240)  # ~12 seconds at 20 Hz

        # ----- Layout -----
        main = ttk.Frame(root, padding=14)
        main.pack(fill="both", expand=True)

        top = ttk.Frame(main)
        top.pack(fill="x")

        # Big status card
        self.status_frame = tk.Frame(top, bd=0, highlightthickness=0)
        self.status_frame.pack(side="left", fill="both", expand=True, padx=(0, 12))

        self.status_label = tk.Label(
            self.status_frame,
            text="NO DATA",
            font=("Arial", 44, "bold"),
            pady=10
        )
        self.status_label.pack(fill="x")

        self.dist_label = tk.Label(
            self.status_frame,
            text="--- m",
            font=("Arial", 38),
            pady=10
        )
        self.dist_label.pack(fill="x")

        self.small_info = tk.Label(
            self.status_frame,
            text=f"Front cone: {FRONT_CENTER_DEG:.0f}° ±{FRONT_HALF_CONE:.0f}°    STOP<{STOP_DIST_M:.2f}m  SLOW<{SLOW_DIST_M:.2f}m",
            font=("Arial", 12)
        )
        self.small_info.pack(fill="x", pady=(6, 0))

        # Right side: proximity bar + legend
        right = ttk.Frame(top)
        right.pack(side="right", fill="y")

        ttk.Label(right, text="Proximity", font=("Arial", 14, "bold")).pack(anchor="w")
        self.bar_canvas = tk.Canvas(right, width=260, height=60, highlightthickness=0)
        self.bar_canvas.pack(pady=(6, 10))
        self.bar_bg = self.bar_canvas.create_rectangle(0, 0, 260, 60, outline="", fill="#222222")
        self.bar_fill = self.bar_canvas.create_rectangle(0, 0, 0, 60, outline="", fill="#00d26a")
        self.bar_text = self.bar_canvas.create_text(130, 30, text="", fill="white", font=("Arial", 14, "bold"))

        self.legend = tk.Label(
            right,
            text="CLEAR: > SLOW\nSLOW : ≤ SLOW\nSTOP : ≤ STOP",
            font=("Arial", 12),
            justify="left"
        )
        self.legend.pack(anchor="w")

        # Bottom: history graph
        ttk.Label(main, text="Front distance history (last ~12s)", font=("Arial", 14, "bold")).pack(anchor="w", pady=(18, 6))
        self.graph = tk.Canvas(main, height=220, highlightthickness=0)
        self.graph.pack(fill="x", expand=False)

        self.last_gui_update = time.time()
        self.root.after(int(1000 / UPDATE_HZ), self.update_gui)

        root.protocol("WM_DELETE_WINDOW", self.on_close)

    def set_status_style(self, state):
        # Background colors for the big label area
        if state == "STOP":
            bg = "#b00020"
            fg = "white"
        elif state == "SLOW":
            bg = "#f5a300"
            fg = "black"
        elif state == "CLEAR":
            bg = "#0b7d3e"
            fg = "white"
        else:
            bg = "#2b2b2b"
            fg = "white"

        self.status_frame.configure(bg=bg)
        self.status_label.configure(bg=bg, fg=fg)
        self.dist_label.configure(bg=bg, fg=fg)
        self.small_info.configure(bg=bg, fg=fg)

        # Proximity bar color
        if state == "STOP":
            self.bar_canvas.itemconfig(self.bar_fill, fill="#ff3b30")
        elif state == "SLOW":
            self.bar_canvas.itemconfig(self.bar_fill, fill="#ffcc00")
        elif state == "CLEAR":
            self.bar_canvas.itemconfig(self.bar_fill, fill="#00d26a")
        else:
            self.bar_canvas.itemconfig(self.bar_fill, fill="#666666")

    def update_bar(self, dist_m, state):
        # Fill bar more when closer. Use DASH_MAX_RANGE_M as full scale.
        w = 260
        if dist_m is None:
            fill_w = 0
            txt = "—"
        else:
            d = max(0.0, min(DASH_MAX_RANGE_M, float(dist_m)))
            # invert: near=full, far=empty
            ratio = 1.0 - (d / DASH_MAX_RANGE_M)
            fill_w = int(w * ratio)
            txt = f"{d:.2f} m"

        self.bar_canvas.coords(self.bar_fill, 0, 0, fill_w, 60)
        self.bar_canvas.itemconfig(self.bar_text, text=txt)

    def draw_history(self):
        self.graph.delete("all")
        W = self.graph.winfo_width()
        H = self.graph.winfo_height()
        if W < 10 or H < 10:
            return

        # Background
        self.graph.create_rectangle(0, 0, W, H, fill="#121212", outline="")

        # Axes + labels
        self.graph.create_text(8, 10, text=f"{DASH_MAX_RANGE_M:.1f}m", fill="#bbbbbb", anchor="w", font=("Arial", 10))
        self.graph.create_text(8, H-10, text="0.0m", fill="#bbbbbb", anchor="w", font=("Arial", 10))

        if len(self.history) < 2:
            return

        # Map distances to y (0 at bottom)
        def y_from_d(d):
            d = max(0.0, min(DASH_MAX_RANGE_M, d))
            return H - (d / DASH_MAX_RANGE_M) * H

        # Build polyline
        xs = np.linspace(0, W, len(self.history))
        pts = []
        for x, d in zip(xs, self.history):
            if d is None:
                # break line: draw gaps
                pts.append(None)
            else:
                pts.append((x, y_from_d(d)))

        # Draw segments (gaps for None)
        last = None
        for p in pts:
            if p is None:
                last = None
                continue
            if last is not None:
                self.graph.create_line(last[0], last[1], p[0], p[1], fill="#4da3ff", width=2)
            last = p

        # Threshold lines
        y_stop = y_from_d(STOP_DIST_M)
        y_slow = y_from_d(SLOW_DIST_M)
        self.graph.create_line(0, y_stop, W, y_stop, fill="#ff3b30", width=1, dash=(6, 4))
        self.graph.create_line(0, y_slow, W, y_slow, fill="#ffcc00", width=1, dash=(6, 4))
        self.graph.create_text(W-6, y_stop-6, text="STOP", fill="#ff3b30", anchor="e", font=("Arial", 10, "bold"))
        self.graph.create_text(W-6, y_slow-6, text="SLOW", fill="#ffcc00", anchor="e", font=("Arial", 10, "bold"))

    def update_gui(self):
        raw, smoothed, ts = self.reader.get()
        now = time.time()

        # If no update for a while, consider it "NO DATA"
        stale = (now - ts) > 1.0
        dist = None if stale else smoothed

        state = classify(dist)

        # Update labels
        self.status_label.config(text=state)
        if dist is None:
            self.dist_label.config(text="--- m")
        else:
            self.dist_label.config(text=f"{dist:.2f} m")

        self.set_status_style(state)
        self.update_bar(dist, state)

        # Save history
        self.history.append(dist)

        # Draw graph
        self.draw_history()

        self.root.after(int(1000 / UPDATE_HZ), self.update_gui)

    def on_close(self):
        self.reader.stop()
        self.root.destroy()


def main():
    root = tk.Tk()
    # Make ttk look nicer on Ubuntu
    try:
        style = ttk.Style()
        style.theme_use("clam")
    except Exception:
        pass
    DashboardGUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()