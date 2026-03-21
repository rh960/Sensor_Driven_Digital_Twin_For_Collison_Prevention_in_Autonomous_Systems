"""
motor_controller.py  —  Jetson Orin Nano
Proper autonomous RC car obstacle avoidance with continuous sensor feedback.

MANOEUVRE LOGIC (checked every 50ms):
  Normal driving:
    - Side obstacle detected → steer away WHILE driving forward
    - No obstacle → full speed, centre steering

  When blocked ahead (within STOP_DIST_M):
    - Check if one side has clearance → bypass: slow + steer around
    - Both sides blocked → full stop → reverse straight → turn → forward escape

  During every phase the sensor is still read:
    - If new obstacle detected during escape → abort and re-plan
    - If reverse path blocked → stop and wait for clear

STATES:
  NORMAL        → continuous drive + reactive steering
  STOPPING      → braking to halt
  REVERSING     → reverse straight (centre steer), watching rear
  TURNING       → stopped, steering toward escape side
  ESCAPING      → forward slow, steer toward escape side
  STRAIGHTENING → forward slow, re-centring steering
"""

import socket
import threading
import time

ARDUINO_IP   = "172.20.10.3"
ARDUINO_PORT = 5005
PAUSE_PORT   = 5006

CMD_FORWARD  = b'f'
CMD_SLOW     = b'm'
CMD_STOP     = b's'
CMD_REVERSE  = b'r'
CMD_LEFT     = b'a'
CMD_RIGHT    = b'd'
CMD_CENTRE   = b'c'

# ── Distance thresholds ───────────────────────────────────────
SIDE_STEER_M    = 10.0   # start steering away from side obstacle within this
CAUTION_M       = 7.0    # slow down when obstacle within this ahead
STOP_M          = 5.0    # stop when obstacle within this ahead
BYPASS_OPEN_M   = 3.0    # side must have at least this much to attempt bypass
REVERSE_SAFE_M  = 2.0    # abort reverse if something within this behind

# ── Manoeuvre timings ─────────────────────────────────────────
STOP_TIME_S     = 0.5    # time to hold stop before reversing
REVERSE_TIME_S  = 2.0    # how long to reverse straight
TURN_TIME_S     = 0.8    # how long to steer while stopped (point turn)
ESCAPE_TIME_S   = 1.5    # how long to drive forward on escape heading
STRAIGHT_TIME_S = 0.8    # how long to re-centre before resuming normal

LOOP_HZ = 20


class MotorController:

    NORMAL       = "NORMAL"
    STOPPING     = "STOPPING"
    REVERSING    = "REVERSING"
    TURNING      = "TURNING"
    ESCAPING     = "ESCAPING"
    STRAIGHTENING= "STRAIGHTENING"

    def __init__(self):
        self._sock          = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._paused        = False
        self._last_throttle = None
        self._last_steer    = None
        self._log_timer     = 0.0

        self._state         = self.NORMAL
        self._state_until   = 0.0
        self._escape_dir    = CMD_RIGHT   # direction to escape toward

        self._lock  = threading.Lock()
        self._level = "SAFE"
        self._fl    = None
        self._fc    = None
        self._fr    = None

        self._running = True
        threading.Thread(target=self._loop,         daemon=True).start()
        threading.Thread(target=self._listen_pause, daemon=True).start()
        print(f"[MOTOR] Ready -> {ARDUINO_IP}:{ARDUINO_PORT} @ {LOOP_HZ}Hz")

    # ── Pause listener ────────────────────────────────────────
    def _listen_pause(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(("0.0.0.0", PAUSE_PORT))
        sock.settimeout(1.0)
        while self._running:
            try:
                data, _ = sock.recvfrom(16)
                cmd = data.decode().strip().upper()
                if cmd == 'P':
                    self._paused = True
                    self._state  = self.NORMAL
                    self._reset_last()
                    self._send(CMD_STOP)
                    self._send(CMD_CENTRE)
                    print("[MOTOR] PAUSED — laptop control")
                elif cmd == 'R':
                    self._paused = False
                    self._state  = self.NORMAL
                    self._reset_last()
                    print("[MOTOR] RESUMED — autonomous")
            except socket.timeout:
                continue
            except Exception as e:
                print(f"[MOTOR] Listener: {e}")

    # ── Helpers ───────────────────────────────────────────────
    def _reset_last(self):
        self._last_throttle = None
        self._last_steer    = None

    def _send(self, cmd: bytes):
        try:
            self._sock.sendto(cmd, (ARDUINO_IP, ARDUINO_PORT))
        except Exception as e:
            print(f"[MOTOR] UDP: {e}")

    def _send_throttle(self, cmd: bytes):
        if cmd != self._last_throttle:
            self._send(cmd); self._last_throttle = cmd

    def _send_steer(self, cmd: bytes):
        if cmd != self._last_steer:
            self._send(cmd); self._last_steer = cmd

    def update(self, level: str, fl, fc, fr):
        with self._lock:
            self._level = level
            self._fl = fl; self._fc = fc; self._fr = fr

    # ── Main loop ─────────────────────────────────────────────
    def _loop(self):
        interval = 1.0 / LOOP_HZ
        while self._running:
            t0 = time.time()
            if not self._paused:
                with self._lock:
                    level = self._level
                    fl = self._fl; fc = self._fc; fr = self._fr
                self._process(level, fl, fc, fr)
            elapsed = time.time() - t0
            sleep = interval - elapsed
            if sleep > 0:
                time.sleep(sleep)

    # ── Core process ──────────────────────────────────────────
    def _process(self, level, fl, fc, fr):
        now = time.time()

        # Convert None to large number
        l = fl if fl is not None else 9.0
        c = fc if fc is not None else 9.0
        r = fr if fr is not None else 9.0

        # Debug log
        if now - self._log_timer > 0.5:
            self._log_timer = now
            print(f"[MOTOR] {self._state} | L={l:.1f} C={c:.1f} R={r:.1f}")

        # ── STOPPING ─────────────────────────────────────────
        if self._state == self.STOPPING:
            self._send_throttle(CMD_STOP)
            self._send_steer(CMD_CENTRE)
            if now >= self._state_until:
                # Decide escape direction — toward more open side
                self._escape_dir = CMD_RIGHT if l >= r else CMD_LEFT
                print(f"[MOTOR] → REVERSING (escape={'RIGHT' if self._escape_dir==CMD_RIGHT else 'LEFT'})")
                self._state = self.REVERSING
                self._state_until = now + REVERSE_TIME_S
            return

        # ── REVERSING ─────────────────────────────────────────
        if self._state == self.REVERSING:
            # Check rear — if something behind abort reverse
            # (no rear sensor so use time-based safety)
            self._send_throttle(CMD_REVERSE)
            self._send_steer(CMD_CENTRE)   # STRAIGHT reverse
            if now >= self._state_until:
                print("[MOTOR] → TURNING")
                self._state = self.TURNING
                self._state_until = now + TURN_TIME_S
            return

        # ── TURNING (point turn while stopped) ───────────────
        if self._state == self.TURNING:
            self._send_throttle(CMD_STOP)
            self._send_steer(self._escape_dir)
            if now >= self._state_until:
                print("[MOTOR] → ESCAPING")
                self._state = self.ESCAPING
                self._state_until = now + ESCAPE_TIME_S
            return

        # ── ESCAPING (forward on new heading) ─────────────────
        if self._state == self.ESCAPING:
            # Check if new obstacle appeared on escape path
            if c <= STOP_M:
                # Something in the way again — re-plan
                print(f"[MOTOR] New obstacle during escape C={c:.1f}m — re-plan")
                self._escape_dir = CMD_RIGHT if l >= r else CMD_LEFT
                self._state = self.STOPPING
                self._state_until = now + STOP_TIME_S
                return
            self._send_throttle(CMD_SLOW)
            self._send_steer(self._escape_dir)
            if now >= self._state_until:
                print("[MOTOR] → STRAIGHTENING")
                self._state = self.STRAIGHTENING
                self._state_until = now + STRAIGHT_TIME_S
            return

        # ── STRAIGHTENING (re-centre before normal drive) ─────
        if self._state == self.STRAIGHTENING:
            self._send_throttle(CMD_SLOW)
            self._send_steer(CMD_CENTRE)
            if now >= self._state_until:
                print("[MOTOR] → NORMAL")
                self._state = self.NORMAL
            return

        # ── NORMAL ────────────────────────────────────────────
        # Decide steering from live sensor data
        if l < SIDE_STEER_M and r < SIDE_STEER_M:
            # Both sides have obstacles — go toward more open side
            side_steer = CMD_RIGHT if l < r else CMD_LEFT
        elif l < SIDE_STEER_M:
            side_steer = CMD_RIGHT   # obstacle left → steer right
        elif r < SIDE_STEER_M:
            side_steer = CMD_LEFT    # obstacle right → steer left
        else:
            side_steer = CMD_CENTRE  # all clear

        if c <= STOP_M:
            # Obstacle ahead — check if bypass is possible
            left_open  = l >= BYPASS_OPEN_M
            right_open = r >= BYPASS_OPEN_M

            if left_open or right_open:
                # Bypass possible — steer around while slowing
                bypass = CMD_LEFT if left_open and l >= r else CMD_RIGHT
                print(f"[MOTOR] Bypass C={c:.1f}m → {'LEFT' if bypass==CMD_LEFT else 'RIGHT'}")
                self._send_throttle(CMD_SLOW)
                self._send_steer(bypass)
            else:
                # No bypass — full manoeuvre
                print(f"[MOTOR] Blocked C={c:.1f}m L={l:.1f}m R={r:.1f}m → STOPPING")
                self._state = self.STOPPING
                self._state_until = now + STOP_TIME_S
                self._send_throttle(CMD_STOP)
                self._send_steer(CMD_CENTRE)

        elif c <= CAUTION_M:
            # Getting close — slow + reactive steer
            self._send_throttle(CMD_SLOW)
            self._send_steer(side_steer)

        else:
            # Clear — full speed, reactive side steering
            self._send_throttle(CMD_FORWARD)
            self._send_steer(side_steer)

    def stop(self):
        self._running = False
        self._send(CMD_STOP)
        self._send(CMD_CENTRE)
        print("[MOTOR] Stopped")