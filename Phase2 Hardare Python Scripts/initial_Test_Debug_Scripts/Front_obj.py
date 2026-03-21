import time
import serial
import numpy as np
from collections import deque

# --------- LD06 packet format ----------
PKT_LEN = 47
HDR0 = 0x54
HDR1 = 0x2C
PTS_PER_PKT = 12

# --------- Jetson serial ----------
PORT = "/dev/ttyTHS1"
BAUD = 230400

# --------- Detection tuning ----------
# "Front" reference:
# The small triangle arrow on the LD06 top cap is your 0° reference.
# Set FRONT_CENTER_DEG to whatever direction is "forward" for your car.
FRONT_CENTER_DEG = 0.0      # degrees
FRONT_HALF_CONE = 25.0      # ±25° (total 50° forward cone)

# If your lidar is mounted rotated relative to the car forward direction,
# add an offset here. Example: if arrow points to the right of the car, offset = -90
ANGLE_OFFSET_DEG = 0.0

STOP_DIST_M = 0.50
SLOW_DIST_M = 1.20

MIN_VALID_M = 0.08
MAX_VALID_M = 12.0

# stability
WINDOW_SEC = 0.25           # consider closest point seen in last 0.25s
PRINT_HZ = 10               # print 10x/sec


def ang_diff_deg(a, b):
    """Smallest signed difference a-b in degrees in [-180,180]."""
    return (a - b + 180.0) % 360.0 - 180.0


def in_front(angle_deg):
    return abs(ang_diff_deg(angle_deg, FRONT_CENTER_DEG)) <= FRONT_HALF_CONE


def decode_packet(pkt: bytes):
    """Return list of (angle_deg, dist_m) or None."""
    if len(pkt) != PKT_LEN:
        return None
    if pkt[0] != HDR0 or pkt[1] != HDR1:
        return None

    start_angle = (pkt[4] | (pkt[5] << 8)) / 100.0

    base = 6
    d_mm = []
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


def classify(min_front):
    if min_front is None:
        return "NO DATA"
    if min_front <= STOP_DIST_M:
        return "STOP"
    if min_front <= SLOW_DIST_M:
        return "SLOW"
    return "CLEAR"


def main():
    print("LD06 Front Obstacle Detector")
    print(f"Port={PORT} Baud={BAUD}")
    print(f"Front cone: {FRONT_CENTER_DEG:.1f}° ±{FRONT_HALF_CONE:.1f}°")
    print(f"STOP<{STOP_DIST_M}m  SLOW<{SLOW_DIST_M}m")
    print("Ctrl+C to exit.\n")

    ser = serial.Serial(PORT, BAUD, timeout=0.2)
    ser.reset_input_buffer()

    buf = bytearray()
    # store recent front distances for stability
    front_hist = deque()  # (t, dist)

    next_print = time.time()

    try:
        while True:
            chunk = ser.read(1024)
            if chunk:
                buf.extend(chunk)

            # prevent runaway buffer
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

            # drop old
            now = time.time()
            cutoff = now - WINDOW_SEC
            while front_hist and front_hist[0][0] < cutoff:
                front_hist.popleft()

            # print status
            if now >= next_print:
                next_print = now + 1.0 / PRINT_HZ

                min_front = None
                if front_hist:
                    min_front = float(min(d for _, d in front_hist))

                state = classify(min_front)
                if min_front is None:
                    print(f"[{state}] front_min = ---")
                else:
                    print(f"[{state}] front_min = {min_front:.2f} m")

    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        ser.close()


if __name__ == "__main__":
    main()