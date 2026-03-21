import time
import threading
from collections import deque

import numpy as np
import serial
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# LD06 packet basics (common format)
PKT_LEN = 47
HDR0 = 0x54
HDR1 = 0x2C
PTS_PER_PKT = 12

def decode_packet(pkt: bytes):
    """
    Decode one LD06 packet (47 bytes) with header 0x54 0x2C.
    Returns list of (angle_deg, dist_m) points, or None.
    """
    if len(pkt) != PKT_LEN:
        return None
    if pkt[0] != HDR0 or pkt[1] != HDR1:
        return None

    start_angle = (pkt[4] | (pkt[5] << 8)) / 100.0

    # 12 points: [dist_lo, dist_hi, intensity] repeated
    d_mm = []
    base = 6
    for i in range(PTS_PER_PKT):
        dist = pkt[base + 3*i] | (pkt[base + 3*i + 1] << 8)
        d_mm.append(dist)

    end_angle = (pkt[42] | (pkt[43] << 8)) / 100.0

    # Handle wrap-around
    sa = start_angle
    ea = end_angle
    if ea < sa:
        ea += 360.0

    angles = np.linspace(sa, ea, PTS_PER_PKT, endpoint=False)
    angles = np.mod(angles, 360.0)

    d_m = np.array(d_mm, dtype=np.float32) / 1000.0

    # Filter (LD06 rated ~0.02–12m; keep a safe range)
    valid = (d_m > 0.05) & (d_m < 12.0)
    pts = [(float(a), float(d)) for a, d, ok in zip(angles, d_m, valid) if ok]
    return pts if pts else None


class SerialReader(threading.Thread):
    def __init__(self, port: str, baud: int, out_deque: deque):
        super().__init__(daemon=True)
        self.port = port
        self.baud = baud
        self.out = out_deque
        self._stop = threading.Event()

    def stop(self):
        self._stop.set()

    def run(self):
        while not self._stop.is_set():
            try:
                with serial.Serial(self.port, self.baud, timeout=0.2) as ser:
                    ser.reset_input_buffer()
                    buf = bytearray()

                    while not self._stop.is_set():
                        chunk = ser.read(1024)
                        if chunk:
                            buf.extend(chunk)
                        else:
                            continue

                        # Prevent runaway buffer
                        if len(buf) > 8192:
                            buf = buf[-4096:]

                        i = 0
                        while i + PKT_LEN <= len(buf):
                            if buf[i] == HDR0 and buf[i + 1] == HDR1:
                                pkt = bytes(buf[i:i + PKT_LEN])
                                pts = decode_packet(pkt)
                                if pts:
                                    self.out.extend(pts)
                                i += PKT_LEN
                            else:
                                i += 1

                        if i > 0:
                            del buf[:i]

            except serial.SerialException as e:
                print(f"[LD06] Serial error: {e} — retrying in 1s")
                time.sleep(1)
            except Exception as e:
                print(f"[LD06] Error: {e} — retrying in 1s")
                time.sleep(1)


def main():
    port = "/dev/ttyTHS1"
    baud = 230400
    max_range_m = 6.0        # change if you want
    trail_points = 2500      # how many recent points to draw

    pts_deque = deque(maxlen=trail_points)
    reader = SerialReader(port, baud, pts_deque)
    reader.start()

    plt.figure("LD06 Radar")
    ax = plt.subplot(111, projection="polar")
    ax.set_theta_zero_location("N")   # 0° up
    ax.set_theta_direction(-1)        # clockwise
    ax.set_rlim(0, max_range_m)
    ax.grid(True)

    scat = ax.scatter([], [], s=6)

    def update(_):
        if not pts_deque:
            return scat,
        pts = list(pts_deque)
        theta = np.deg2rad([p[0] for p in pts])
        r = np.array([p[1] for p in pts], dtype=np.float32)
        scat.set_offsets(np.c_[theta, r])
        return scat,

    ani = FuncAnimation(plt.gcf(), update, interval=50, blit=False)

    try:
        plt.show()
    finally:
        reader.stop()


if __name__ == "__main__":
    main()