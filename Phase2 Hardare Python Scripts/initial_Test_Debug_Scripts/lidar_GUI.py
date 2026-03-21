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

# ---------------- Front sectors (degrees) ----------------
# The arrow on LD06 top cap is your 0° reference.
# If arrow points forward, leave OFFSET=0.
ANGLE_OFFSET_DEG = 0.0

# Define three forward sectors:
#   LEFT  : +10° to +35°
#   CENTER: -10° to +10°
#   RIGHT : -35° to -10°
# You can tune these.
CENTER_HALF = 10.0
SIDE_INNER = 10.0
SIDE_OUTER = 35.0

# Thresholds (meters) - tune for your RC car
STOP_DIST_M = 0.80
SLOW_DIST_M = 1.80

# Valid range
MIN_VALID_M = 0.08
MAX_VALID_M = 12.0

# Stability/smoothing
WINDOW_SEC = 0.25
SMOOTH_ALPHA = 0.35
UPDATE_HZ = 20

# Gauge scale (what full-scale means)
GAUGE_MAX_M = 3.0


def ang_diff_deg(a, b=0.0):
    """Smallest signed difference a-b in degrees in [-180,180]."""
    return (a - b + 180.0) % 360.0 - 180.0


def classify(dist_m):
    if dist_m is None:
        return "NO DATA"
    if dist_m <= STOP_DIST_M:
        return "STOP"
    if dist_m <= SLOW_DIST_M:
        return "SLOW"
    return "CLEAR"


def decode_packet(pkt: bytes):
    if len(pkt) != PKT_LEN:
        return None
    if pkt[0] != HDR0 or pkt[1] != HDR1:
        return None

    start_angle = (pkt[4] | (pkt[5] << 8)) / 100.0

    d_mm = []
    base = 6
    for i in range(PTS_PER_PKT):
        dist = pkt[base + 3 * i] | (pkt[base + 3 * i + 1] << 8)
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


def sector_of(angle_deg):
    """
    Use signed angle around 0° (front).
    + is left, - is right.
    """
    s = ang_diff_deg(angle_deg, 0.0)  # -180..180

    if abs(s) <= CENTER_HALF:
        return "C"
    if SIDE_INNER < s <= SIDE_OUTER:
        return "L"
    if -SIDE_OUTER <= s < -SIDE_INNER:
        return "R"
    return None


class LD06FrontLRReader(threading.Thread):
    """
    Continuously updates min distances for Left/Center/Right in the last WINDOW_SEC.
    """
    def __init__(self, port, baud):
        super().__init__(daemon=True)
        self.port = port
        self.baud = baud
        self.stop_flag = threading.Event()

        self._lock = threading.Lock()
        self.raw_L = None
        self.raw_C = None
        self.raw_R = None
        self.smooth_L = None
        self.smooth_C = None
        self.smooth_R = None
        self.last_update_ts = 0.0

    def get(self):
        with self._lock:
            return (self.raw_L, self.raw_C, self.raw_R,
                    self.smooth_L, self.smooth_C, self.smooth_R,
                    self.last_update_ts)

    def stop(self):
        self.stop_flag.set()

    def run(self):
        while not self.stop_flag.is_set():
            try:
                ser = serial.Serial(self.port, self.baud, timeout=0.2)
                ser.reset_input_buffer()
                buf = bytearray()

                # store (t, dist) per sector
                hist_L = deque()
                hist_C = deque()
                hist_R = deque()

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
                                    sec = sector_of(ang)
                                    if sec == "L":
                                        hist_L.append((now, dist))
                                    elif sec == "C":
                                        hist_C.append((now, dist))
                                    elif sec == "R":
                                        hist_R.append((now, dist))
                            i += PKT_LEN
                        else:
                            i += 1

                    if i > 0:
                        del buf[:i]

                    now = time.time()
                    cutoff = now - WINDOW_SEC

                    while hist_L and hist_L[0][0] < cutoff:
                        hist_L.popleft()
                    while hist_C and hist_C[0][0] < cutoff:
                        hist_C.popleft()
                    while hist_R and hist_R[0][0] < cutoff:
                        hist_R.popleft()

                    cur_L = float(min(d for _, d in hist_L)) if hist_L else None
                    cur_C = float(min(d for _, d in hist_C)) if hist_C else None
                    cur_R = float(min(d for _, d in hist_R)) if hist_R else None

                    with self._lock:
                        self.raw_L, self.raw_C, self.raw_R = cur_L, cur_C, cur_R
                        self.last_update_ts = now

                        # smoothing per channel (EMA)
                        def ema(prev, cur):
                            if cur is None:
                                return prev
                            if prev is None:
                                return cur
                            a = SMOOTH_ALPHA
                            return a * cur + (1 - a) * prev

                        self.smooth_L = ema(self.smooth_L, cur_L)
                        self.smooth_C = ema(self.smooth_C, cur_C)
                        self.smooth_R = ema(self.smooth_R, cur_R)

                ser.close()

            except Exception:
                time.sleep(1.0)


class Dashboard:
    def __init__(self, root):
        self.root = root
        root.title("LD06 Front Dashboard (Gauge + L/R)")
        root.geometry("980x560")

        self.reader = LD06FrontLRReader(PORT, BAUD)
        self.reader.start()

        main = ttk.Frame(root, padding=14)
        main.pack(fill="both", expand=True)

        top = ttk.Frame(main)
        top.pack(fill="x")

        # Left panel: Gauge
        gauge_frame = ttk.Frame(top)
        gauge_frame.pack(side="left", fill="both", expand=True, padx=(0, 14))

        ttk.Label(gauge_frame, text="Front Distance", font=("Arial", 16, "bold")).pack(anchor="w")

        self.gauge = tk.Canvas(gauge_frame, width=520, height=420, highlightthickness=0)
        self.gauge.pack(pady=(8, 0))

        # Right panel: Left/Right sensors
        lr_frame = ttk.Frame(top)
        lr_frame.pack(side="right", fill="y")

        ttk.Label(lr_frame, text="Parking Sensors", font=("Arial", 16, "bold")).pack(anchor="w")

        self.card_L = self.make_card(lr_frame, "LEFT")
        self.card_C = self.make_card(lr_frame, "CENTER")
        self.card_R = self.make_card(lr_frame, "RIGHT")

        self.card_L.pack(fill="x", pady=(10, 6))
        self.card_C.pack(fill="x", pady=6)
        self.card_R.pack(fill="x", pady=6)

        # Bottom: status bar
        bottom = ttk.Frame(main)
        bottom.pack(fill="x", pady=(14, 0))

        self.status_big = tk.Label(bottom, text="NO DATA", font=("Arial", 22, "bold"), pady=8)
        self.status_big.pack(fill="x")

        self.help = ttk.Label(
            main,
            text=f"Sectors: L=+{SIDE_INNER:.0f}°..+{SIDE_OUTER:.0f}°  "
                 f"C=±{CENTER_HALF:.0f}°  "
                 f"R=-{SIDE_OUTER:.0f}°..-{SIDE_INNER:.0f}°    "
                 f"STOP<{STOP_DIST_M:.2f}m  SLOW<{SLOW_DIST_M:.2f}m",
            font=("Arial", 11)
        )
        self.help.pack(anchor="w", pady=(10, 0))

        # Pre-draw gauge
        self.gauge_items = self.draw_gauge_static()

        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.root.after(int(1000 / UPDATE_HZ), self.update_ui)

    def make_card(self, parent, title):
        f = tk.Frame(parent, bd=0)
        f.configure(bg="#2b2b2b")
        lbl_title = tk.Label(f, text=title, font=("Arial", 14, "bold"), bg="#2b2b2b", fg="white")
        lbl_title.pack(anchor="w", padx=12, pady=(10, 0))
        lbl_val = tk.Label(f, text="--- m", font=("Arial", 22), bg="#2b2b2b", fg="white")
        lbl_val.pack(anchor="w", padx=12, pady=(4, 10))
        f._val = lbl_val
        return f

    def set_card_state(self, card, state, dist):
        if state == "STOP":
            bg = "#b00020"; fg = "white"
        elif state == "SLOW":
            bg = "#f5a300"; fg = "black"
        elif state == "CLEAR":
            bg = "#0b7d3e"; fg = "white"
        else:
            bg = "#2b2b2b"; fg = "white"

        card.configure(bg=bg)
        for w in card.winfo_children():
            w.configure(bg=bg, fg=fg)

        if dist is None:
            card._val.configure(text="--- m")
        else:
            card._val.configure(text=f"{dist:.2f} m")

    def draw_gauge_static(self):
        c = self.gauge
        c.delete("all")

        W = int(c["width"]); H = int(c["height"])
        cx, cy = W // 2, H // 2 + 40

        # Gauge arc bounds
        r = 180
        bbox = (cx - r, cy - r, cx + r, cy + r)

        # Background
        c.create_rectangle(0, 0, W, H, fill="#111111", outline="")

        # Draw arc (like a speedometer): from 210° to -30° (i.e. 240° sweep)
        # Tk angles: 0° is at 3 o'clock, positive is counterclockwise.
        # We'll use start=210 and extent=240.
        c.create_arc(bbox, start=210, extent=240, style="arc", width=18, outline="#444444")

        # Tick marks + labels
        for i in range(0, 7):  # 0..6
            frac = i / 6.0
            val = GAUGE_MAX_M * frac
            ang = self.gauge_angle_for_value(val)
            x1, y1 = self.polar(cx, cy, r - 8, ang)
            x2, y2 = self.polar(cx, cy, r - 30, ang)
            c.create_line(x1, y1, x2, y2, fill="#bbbbbb", width=3)
            xl, yl = self.polar(cx, cy, r - 55, ang)
            c.create_text(xl, yl, text=f"{val:.1f}", fill="#dddddd", font=("Arial", 11, "bold"))

        # Center text placeholders
        title_id = c.create_text(cx, cy - 110, text="FRONT", fill="#dddddd", font=("Arial", 16, "bold"))
        dist_id = c.create_text(cx, cy - 60, text="--- m", fill="white", font=("Arial", 32, "bold"))
        state_id = c.create_text(cx, cy - 20, text="NO DATA", fill="#bbbbbb", font=("Arial", 20, "bold"))

        # Needle
        needle_id = c.create_line(cx, cy, cx, cy - (r - 50), fill="#ff4d4d", width=6)

        # Hub
        hub_id = c.create_oval(cx - 10, cy - 10, cx + 10, cy + 10, fill="#dddddd", outline="")

        return {
            "cx": cx, "cy": cy, "r": r,
            "dist_id": dist_id, "state_id": state_id,
            "needle_id": needle_id
        }

    def gauge_angle_for_value(self, val_m):
        """
        Map distance [0..GAUGE_MAX_M] to gauge angle in degrees (Tk polar angle),
        where 0m = left/red side, max = right side (like a speedometer but reversed).
        We'll map:
          val=0   -> angle at ~210° (left)
          val=max -> angle at ~-30° (right) => 330°
        """
        v = max(0.0, min(GAUGE_MAX_M, float(val_m)))
        frac = v / GAUGE_MAX_M
        # sweep 240° from 210 to 450(=90)?? but we want 210->330 (which is +120)
        # Better: use the same arc we drew: start=210 extent=240 ends at 450 (=90).
        # We'll place needle along that 240° sweep:
        ang = 210 + 240 * frac
        return ang

    @staticmethod
    def polar(cx, cy, radius, ang_deg):
        # Convert Tk arc angle (0° at 3 o'clock, CCW positive) to x,y
        rad = np.deg2rad(ang_deg)
        x = cx + radius * np.cos(rad)
        y = cy - radius * np.sin(rad)
        return x, y

    def update_gauge(self, dist, state):
        c = self.gauge
        cx = self.gauge_items["cx"]
        cy = self.gauge_items["cy"]
        r = self.gauge_items["r"]

        # Update center text
        if dist is None:
            c.itemconfig(self.gauge_items["dist_id"], text="--- m")
        else:
            c.itemconfig(self.gauge_items["dist_id"], text=f"{dist:.2f} m")

        # Color by state
        if state == "STOP":
            col = "#ff3b30"
        elif state == "SLOW":
            col = "#ffcc00"
        elif state == "CLEAR":
            col = "#00d26a"
        else:
            col = "#bbbbbb"

        c.itemconfig(self.gauge_items["state_id"], text=state, fill=col)

        # Needle angle: if no data, park in middle
        if dist is None:
            ang = self.gauge_angle_for_value(GAUGE_MAX_M * 0.5)
        else:
            ang = self.gauge_angle_for_value(min(dist, GAUGE_MAX_M))

        x2, y2 = self.polar(cx, cy, r - 50, ang)
        c.coords(self.gauge_items["needle_id"], cx, cy, x2, y2)

    def update_ui(self):
        raw_L, raw_C, raw_R, sL, sC, sR, ts = self.reader.get()
        now = time.time()
        stale = (now - ts) > 1.0

        L = None if stale else sL
        C = None if stale else sC
        R = None if stale else sR

        stL = classify(L)
        stC = classify(C)
        stR = classify(R)

        # Cards
        self.set_card_state(self.card_L, stL, L)
        self.set_card_state(self.card_C, stC, C)
        self.set_card_state(self.card_R, stR, R)

        # Big overall state: prioritize STOP > SLOW > CLEAR, using center then sides
        overall = "NO DATA"
        if not stale and (C is not None or L is not None or R is not None):
            if stC == "STOP" or stL == "STOP" or stR == "STOP":
                overall = "STOP"
            elif stC == "SLOW" or stL == "SLOW" or stR == "SLOW":
                overall = "SLOW"
            else:
                overall = "CLEAR"

        # Big status bar color
        if overall == "STOP":
            bg, fg = "#b00020", "white"
        elif overall == "SLOW":
            bg, fg = "#f5a300", "black"
        elif overall == "CLEAR":
            bg, fg = "#0b7d3e", "white"
        else:
            bg, fg = "#2b2b2b", "white"

        self.status_big.configure(text=f"{overall}   (Front: {('---' if C is None else f'{C:.2f} m')})", bg=bg, fg=fg)

        # Gauge uses CENTER distance
        self.update_gauge(C, stC if not stale else "NO DATA")

        self.root.after(int(1000 / UPDATE_HZ), self.update_ui)

    def on_close(self):
        self.reader.stop()
        self.root.destroy()


def main():
    root = tk.Tk()
    try:
        style = ttk.Style()
        style.theme_use("clam")
    except Exception:
        pass
    Dashboard(root)
    root.mainloop()


if __name__ == "__main__":
    main()