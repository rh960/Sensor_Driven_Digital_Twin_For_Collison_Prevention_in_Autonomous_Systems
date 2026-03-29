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
SIDE_STEER_M    = 2.0   # steer away from side obstacles within this
CAUTION_M       = 2.0   # slow down when obstacle within this
STOP_M          = 1.0   # trigger full stop+manoeuvre within this
BYPASS_OPEN_M   = 1.0   # side must be this clear for bypass attempt

# ── Manoeuvre timings ─────────────────────────────────────────
STOP_TIME_S     = 0.5   # hold stop before reversing
REVERSE_TIME_S  = 2.5   # reverse duration — increased for more clearance
TURN_TIME_S     = 1.2   # turning duration — increased for better angle change
ESCAPE_TIME_S   = 2.0   # forward escape duration
STRAIGHT_TIME_S = 0.8   # re-centre before resuming

LOOP_HZ = 20


class MotorController:

    NORMAL        = "NORMAL"
    STOPPING      = "STOPPING"
    REVERSING     = "REVERSING"
    TURNING       = "TURNING"
    ESCAPING      = "ESCAPING"
    STRAIGHTENING = "STRAIGHTENING"

    def __init__(self):
        self._sock          = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._paused        = False
        self._last_throttle = None
        self._last_steer    = None
        self._log_timer     = 0.0
        self._state         = self.NORMAL
        self._state_until   = 0.0
        self._escape_dir    = CMD_RIGHT
        self._manoeuvre_start = 0.0   # tracks when full manoeuvre began
        self._lock          = threading.Lock()
        self._level         = "SAFE"
        self._fl            = None
        self._fc            = None
        self._fr            = None
        self._running       = True
        threading.Thread(target=self._loop,         daemon=True).start()
        threading.Thread(target=self._listen_pause, daemon=True).start()
        print(f"[MOTOR] Ready -> {ARDUINO_IP}:{ARDUINO_PORT} @ {LOOP_HZ}Hz UDP")

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
                    print("[MOTOR] PAUSED - laptop control")
                elif cmd == 'R':
                    self._paused = False
                    self._state  = self.NORMAL
                    self._reset_last()
                    print("[MOTOR] RESUMED - autonomous")
            except socket.timeout:
                continue
            except Exception as e:
                print(f"[MOTOR] Listener: {e}")

    def _reset_last(self):
        self._last_throttle = None
        self._last_steer    = None

    def _send(self, cmd: bytes):
        try:
            self._sock.sendto(cmd, (ARDUINO_IP, ARDUINO_PORT))
        except Exception as e:
            print(f"[MOTOR] UDP error: {e}")

    def _send_throttle(self, cmd: bytes):
        self._send(cmd)
        self._last_throttle = cmd

    def _send_steer(self, cmd: bytes):
        self._send(cmd)
        self._last_steer = cmd

    def update(self, level: str, fl, fc, fr):
        with self._lock:
            self._level = level
            self._fl    = fl
            self._fc    = fc
            self._fr    = fr

    # ── Main loop ─────────────────────────────────────────────
    def _loop(self):
        interval = 1.0 / LOOP_HZ
        while self._running:
            t0 = time.time()
            if not self._paused:
                with self._lock:
                    level = self._level
                    fl    = self._fl
                    fc    = self._fc
                    fr    = self._fr
                self._process(level, fl, fc, fr)
            elapsed = time.time() - t0
            sleep   = interval - elapsed
            if sleep > 0:
                time.sleep(sleep)

    # ── Core state machine ────────────────────────────────────
    def _process(self, level, fl, fc, fr):
        now = time.time()
        l = fl if fl is not None else 9.0
        c = fc if fc is not None else 9.0
        r = fr if fr is not None else 9.0

        if now - self._log_timer > 0.5:
            self._log_timer = now
            print(f"[MOTOR] {self._state} | L={l:.1f} C={c:.1f} R={r:.1f}")

        # ── STOPPING ─────────────────────────────────────────
        if self._state == self.STOPPING:
            self._send_throttle(CMD_STOP)
            self._send_steer(CMD_CENTRE)
            if now >= self._state_until:
                # Pick escape direction toward more open side
                self._escape_dir    = CMD_RIGHT if l >= r else CMD_LEFT
                self._manoeuvre_start = now
                print(f"[MOTOR] -> REVERSING (escape={'RIGHT' if self._escape_dir==CMD_RIGHT else 'LEFT'})")
                self._state       = self.REVERSING
                self._state_until = now + REVERSE_TIME_S
            return

        # ── REVERSING ─────────────────────────────────────────
        if self._state == self.REVERSING:
            # Send reverse every tick — Arduino handles arming internally
            self._send_throttle(CMD_REVERSE)
            self._send_steer(CMD_CENTRE)  # straight reverse
            if now >= self._state_until:
                print("[MOTOR] -> TURNING")
                self._state       = self.TURNING
                self._state_until = now + TURN_TIME_S
            return

        # ── TURNING ───────────────────────────────────────────
        if self._state == self.TURNING:
            # Slow forward while steering — moving turn
            self._send_throttle(CMD_SLOW)
            self._send_steer(self._escape_dir)
            if now >= self._state_until:
                print("[MOTOR] -> ESCAPING")
                self._state       = self.ESCAPING
                self._state_until = now + ESCAPE_TIME_S
            return

        # ── ESCAPING ──────────────────────────────────────────
        if self._state == self.ESCAPING:
            # Only abort escape if literally about to collide (20cm)
            # Do NOT check STOP_M here — that would immediately re-trigger
            if c <= 0.20:
                print(f"[MOTOR] Emergency abort escape C={c:.1f}m")
                self._escape_dir  = CMD_RIGHT if l >= r else CMD_LEFT
                self._state       = self.STOPPING
                self._state_until = now + STOP_TIME_S
                return
            self._send_throttle(CMD_SLOW)
            self._send_steer(self._escape_dir)
            if now >= self._state_until:
                print("[MOTOR] -> STRAIGHTENING")
                self._state       = self.STRAIGHTENING
                self._state_until = now + STRAIGHT_TIME_S
            return

        # ── STRAIGHTENING ─────────────────────────────────────
        if self._state == self.STRAIGHTENING:
            self._send_throttle(CMD_SLOW)
            self._send_steer(CMD_CENTRE)
            if now >= self._state_until:
                print("[MOTOR] -> NORMAL")
                self._state = self.NORMAL
            return

        # ── NORMAL ────────────────────────────────────────────
        # Reactive side steering
        if l < SIDE_STEER_M and r < SIDE_STEER_M:
            side_steer = CMD_RIGHT if l < r else CMD_LEFT
        elif l < SIDE_STEER_M:
            side_steer = CMD_RIGHT
        elif r < SIDE_STEER_M:
            side_steer = CMD_LEFT
        else:
            side_steer = CMD_CENTRE

        if c <= STOP_M:
            left_open  = l >= BYPASS_OPEN_M
            right_open = r >= BYPASS_OPEN_M
            if left_open or right_open:
                # Bypass — steer around while slowing
                bypass = CMD_LEFT if left_open and l >= r else CMD_RIGHT
                print(f"[MOTOR] Bypass C={c:.1f}m -> {'LEFT' if bypass==CMD_LEFT else 'RIGHT'}")
                self._send_throttle(CMD_SLOW)
                self._send_steer(bypass)
            else:
                # Both sides blocked — full manoeuvre
                print(f"[MOTOR] Blocked C={c:.1f}m L={l:.1f}m R={r:.1f}m -> STOPPING")
                self._state       = self.STOPPING
                self._state_until = now + STOP_TIME_S
                self._send_throttle(CMD_STOP)
                self._send_steer(CMD_CENTRE)
        elif c <= CAUTION_M:
            self._send_throttle(CMD_SLOW)
            self._send_steer(side_steer)
        else:
            self._send_throttle(CMD_FORWARD)
            self._send_steer(side_steer)

    def stop(self):
        self._running = False
        self._send(CMD_STOP)
        self._send(CMD_CENTRE)
        print("[MOTOR] Stopped")
