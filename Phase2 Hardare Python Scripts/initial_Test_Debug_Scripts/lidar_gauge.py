import time
import threading
from collections import deque
import numpy as np
import serial
import tkinter as tk

# ---------- LD06 ----------
PKT_LEN = 47
HDR0 = 0x54
HDR1 = 0x2C
PTS_PER_PKT = 12

PORT = "/dev/ttyTHS1"
BAUD = 230400

# ---------- FRONT SECTORS ----------
ANGLE_OFFSET = 0.0   # 0 if arrow points forward

CENTER_HALF = 10.0
SIDE_INNER = 10.0
SIDE_OUTER = 35.0

STOP_DIST = 0.8
SLOW_DIST = 1.8
MAX_RANGE = 3.0

# ---------- UTILS ----------

def ang_diff(a):
    return (a + 180) % 360 - 180


def classify(d):
    if d is None: return "NO DATA"
    if d <= STOP_DIST: return "STOP"
    if d <= SLOW_DIST: return "SLOW"
    return "CLEAR"


def sector(angle):
    s = ang_diff(angle)
    if abs(s) <= CENTER_HALF:
        return "C"
    if SIDE_INNER < s <= SIDE_OUTER:
        return "L"
    if -SIDE_OUTER <= s < -SIDE_INNER:
        return "R"
    return None


# ---------- LD06 DECODER ----------

def decode(pkt):
    if pkt[0] != HDR0 or pkt[1] != HDR1:
        return None

    sa = (pkt[4] | pkt[5] << 8) / 100.0
    ea = (pkt[42] | pkt[43] << 8) / 100.0
    if ea < sa: ea += 360

    angles = np.linspace(sa, ea, PTS_PER_PKT, endpoint=False)
    angles = (angles + ANGLE_OFFSET) % 360

    d = []
    base = 6
    for i in range(PTS_PER_PKT):
        mm = pkt[base + 3*i] | pkt[base + 3*i + 1] << 8
        m = mm / 1000.0
        if 0.08 <= m <= 12:
            d.append((angles[i], m))
    return d


# ---------- SERIAL THREAD ----------

class Reader(threading.Thread):
    def __init__(self):
        super().__init__(daemon=True)
        self.stop = False
        self.L = self.C = self.R = None

    def run(self):
        ser = serial.Serial(PORT, BAUD, timeout=0.2)
        buf = bytearray()
        hist = {"L": deque(), "C": deque(), "R": deque()}

        while not self.stop:
            buf.extend(ser.read(1024))

            i = 0
            while i + PKT_LEN <= len(buf):
                if buf[i] == HDR0 and buf[i+1] == HDR1:
                    pts = decode(buf[i:i+PKT_LEN])
                    if pts:
                        now = time.time()
                        for a, dist in pts:
                            s = sector(a)
                            if s:
                                hist[s].append((now, dist))
                    i += PKT_LEN
                else:
                    i += 1

            if i: del buf[:i]

            cutoff = time.time() - 0.25
            for k in hist:
                while hist[k] and hist[k][0][0] < cutoff:
                    hist[k].popleft()

            self.L = min([d for _, d in hist["L"]], default=None)
            self.C = min([d for _, d in hist["C"]], default=None)
            self.R = min([d for _, d in hist["R"]], default=None)


# ---------- GUI ----------

class Gauge:
    def __init__(self, canvas, x, y, label):
        self.c = canvas
        self.cx = x
        self.cy = y
        self.r = 120

        canvas.create_text(x, y-160, text=label,
                           fill="white", font=("Arial", 16, "bold"))

        self.needle = canvas.create_line(x, y, x, y-100,
                                         width=5, fill="red")
        self.text = canvas.create_text(x, y+40,
                                       fill="white",
                                       font=("Arial", 18, "bold"))

    def update(self, dist):
        if dist is None:
            val = MAX_RANGE / 2
            txt = "---"
        else:
            val = min(dist, MAX_RANGE)
            txt = f"{dist:.2f} m"

        frac = val / MAX_RANGE
        angle = 210 + frac * 240

        rad = np.deg2rad(angle)
        x = self.cx + self.r * np.cos(rad)
        y = self.cy - self.r * np.sin(rad)

        self.c.coords(self.needle, self.cx, self.cy, x, y)
        self.c.itemconfig(self.text, text=txt)


class Dashboard:
    def __init__(self, root):
        self.root = root
        root.title("Forward Object Detection Dashboard")
        root.geometry("1000x550")

        self.canvas = tk.Canvas(root, bg="#111111")
        self.canvas.pack(fill="both", expand=True)

        self.gL = Gauge(self.canvas, 180, 330, "FRONT-LEFT")
        self.gC = Gauge(self.canvas, 500, 330, "FRONT-CENTER")
        self.gR = Gauge(self.canvas, 820, 330, "FRONT-RIGHT")

        self.status = self.canvas.create_text(
            500, 80,
            fill="white",
            font=("Arial", 34, "bold")
        )

        self.reader = Reader()
        self.reader.start()

        self.update()

    def update(self):
        L = self.reader.L
        C = self.reader.C
        R = self.reader.R

        self.gL.update(L)
        self.gC.update(C)
        self.gR.update(R)

        states = [classify(x) for x in (L, C, R) if x is not None]

        if "STOP" in states:
            st, col = "STOP", "#ff3b30"
        elif "SLOW" in states:
            st, col = "SLOW", "#ffcc00"
        else:
            st, col = "CLEAR", "#00d26a"

        self.canvas.itemconfig(self.status,
                               text=f"STATUS: {st}",
                               fill=col)

        self.root.after(50, self.update)


# ---------- MAIN ----------

root = tk.Tk()
Dashboard(root)
root.mainloop()