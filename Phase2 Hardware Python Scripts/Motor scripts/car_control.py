
"""car_control.py  —  runs on laptop
Keyboard control of RC car via Arduino WiFi UDP.

Sends motor commands directly to Arduino on port 5005.
Sends pause/resume to Jetson on port 5006.

Press P to pause Jetson autonomous drive and take manual control.
Press R to resume Jetson autonomous drive.

W/UP    = Forward
S/DOWN  = Reverse
A/LEFT  = Steer Left
D/RIGHT = Steer Right
SPACE   = Stop
P       = Pause Jetson autonomous (take manual control)
R       = Resume Jetson autonomous
Q/ESC   = Quit
"""

import socket
import msvcrt
import time

ARDUINO_IP   = "172.20.10.3"
ARDUINO_PORT = 5005

JETSON_IP    = "172.20.10.2"
JETSON_PORT  = 5006

arduino = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
jetson  = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

paused = False

def send_motor(cmd):
    arduino.sendto(cmd, (ARDUINO_IP, ARDUINO_PORT))

def send_jetson(cmd):
    try:
        jetson.sendto(cmd, (JETSON_IP, JETSON_PORT))
    except:
        pass

def show(t, s, mode):
    print(f"\r  [{mode}]  Throttle: {t:<12} Steering: {s:<10}", end="", flush=True)

print("=" * 50)
print("        RC CAR KEYBOARD CONTROL")
print("=" * 50)
print("  W/UP    = Forward")
print("  S/DOWN  = Reverse")
print("  A/LEFT  = Steer Left")
print("  D/RIGHT = Steer Right")
print("  SPACE   = Stop")
print("  P       = Pause Jetson (manual control)")
print("  R       = Resume Jetson (autonomous)")
print("  Q/ESC   = Quit")
print("=" * 50)
print("\nPlug in ESC battery and wait 5 seconds\n")

throttle_state = "STOP"
steer_state    = "CENTRE"
mode           = "AUTONOMOUS"

try:
    while True:
        if msvcrt.kbhit():
            key = msvcrt.getch()

            # Arrow keys
            if key == b'\xe0':
                key2 = msvcrt.getch()
                if   key2 == b'H': key = b'w'
                elif key2 == b'P': key = b's'
                elif key2 == b'K': key = b'a'
                elif key2 == b'M': key = b'd'

            # Pause Jetson — take manual control
            if key in (b'p', b'P'):
                paused = True
                mode   = "MANUAL"
                send_jetson(b'P')
                send_motor(b's')  # stop car when taking over
                send_motor(b'c')
                throttle_state = "STOP"
                steer_state    = "CENTRE"
                print(f"\n  MANUAL CONTROL - Jetson paused\n")

            # Resume Jetson autonomous
            elif key in (b'r', b'R'):
                paused = False
                mode   = "AUTONOMOUS"
                send_jetson(b'R')
                send_motor(b's')
                send_motor(b'c')
                throttle_state = "STOP"
                steer_state    = "CENTRE"
                print(f"\n  AUTONOMOUS - Jetson resumed\n")

            # Manual control only works when paused
            elif paused:
                if key in (b'w', b'W'):
                    send_motor(b'f')
                    throttle_state = "FORWARD"
                elif key in (b's', b'S'):
                    send_motor(b'r')
                    throttle_state = "REVERSE"
                elif key == b' ':
                    send_motor(b's')
                    throttle_state = "STOP"
                elif key in (b'a', b'A'):
                    send_motor(b'a')
                    steer_state = "LEFT"
                elif key in (b'd', b'D'):
                    send_motor(b'd')
                    steer_state = "RIGHT"
                elif key in (b'c', b'C'):
                    send_motor(b'c')
                    steer_state = "CENTRE"
                elif key in (b'q', b'Q', b'\x1b'):
                    break

            elif key in (b'q', b'Q', b'\x1b'):
                break

            show(throttle_state, steer_state, mode)

        time.sleep(0.02)

finally:
    send_motor(b's')
    send_motor(b'c')
    send_jetson(b'R')  # always resume Jetson on exit
    arduino.close()
    jetson.close()
    print("\n\nStopped. Jetson autonomous resumed.")
