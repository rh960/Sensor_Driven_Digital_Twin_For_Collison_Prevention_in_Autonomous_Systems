import socket
import msvcrt
import time

ARDUINO_IP   = "172.20.10.3"
ARDUINO_PORT = 5005

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

print("=" * 40)
print("       RC CAR WIFI CONTROL")
print("=" * 40)
print("  W / UP    = Forward")
print("  S / DOWN  = Reverse")
print("  A / LEFT  = Steer Left")
print("  D / RIGHT = Steer Right")
print("  SPACE     = Stop")
print("  Q         = Quit")
print("=" * 40 + "\n")

throttle_state = "STOP"
steer_state    = "CENTRE"

def send(cmd):
    sock.sendto(cmd, (ARDUINO_IP, ARDUINO_PORT))

def show(t, s):
    print(f"\r  Throttle: {t:<10}  Steering: {s:<10}", end="", flush=True)

try:
    while True:
        if msvcrt.kbhit():
            key = msvcrt.getch()

            if key == b'\xe0':
                key2 = msvcrt.getch()
                if   key2 == b'H': key = b'w'
                elif key2 == b'P': key = b's'
                elif key2 == b'K': key = b'a'
                elif key2 == b'M': key = b'd'

            if key in (b'w', b'W'):
                send(b'f')
                throttle_state = "FORWARD"
            elif key in (b's', b'S'):
                send(b'r')
                throttle_state = "REVERSE"
            elif key == b' ':
                send(b's')
                throttle_state = "STOP"
            elif key in (b'a', b'A'):
                send(b'a')
                steer_state = "LEFT"
            elif key in (b'd', b'D'):
                send(b'd')
                steer_state = "RIGHT"
            elif key in (b'c', b'C'):
                send(b'c')
                steer_state = "CENTRE"
            elif key in (b'q', b'Q', b'\x1b'):
                send(b's')
                send(b'c')
                print("\n\nStopped. Bye.")
                break

            show(throttle_state, steer_state)

        time.sleep(0.02)

finally:
    send(b's')
    sock.close()