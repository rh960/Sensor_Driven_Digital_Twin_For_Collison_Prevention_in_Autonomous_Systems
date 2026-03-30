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
SIDE_STEER_M    = 2.0
CAUTION_M       = 2.0
STOP_M          = 1.0
BYPASS_OPEN_M   = 1.0

# ── Manoeuvre timings ─────────────────────────────────────────
STOP_TIME_S     = 0.5
REVERSE_TIME_S  = 2.5
TURN_TIME_S     = 1.2
ESCAPE_TIME_S   = 2.0
STRAIGHT_TIME_S = 0.8

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
        self._manoeuvre_start = 0.0
        self._lock          = threading.Lock()
        self._level         = "SAFE"
        self._fl            = None
        self._fc            = None
        self._fr            = None
        self._running       = True

        # ✅ Anti-stuck additions
        self._retry_count = 0
        self._max_retries = 3

        threading.Thread(target=self._loop,         daemon=True).start()
        threading.Thread(target=self._listen_pause, daemon=True).start()
        print(f"[MOTOR] Ready -> {ARDUINO_IP}:{ARDUINO_PORT} @ {LOOP_HZ}Hz UDP")

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

    def _process(self, level, fl, fc, fr):
        now = time.time()
        l = fl if fl is not None else 9.0
        c = fc if fc is not None else 9.0
        r = fr if fr is not None else 9.0

        if now - self._log_timer > 0.5:
            self._log_timer = now
            print(f"[MOTOR] {self._state} | L={l:.1f} C={c:.1f} R={r:.1f}")

        # STOPPING
        if self._state == self.STOPPING:
            self._send_throttle(CMD_STOP)
            self._send_steer(CMD_CENTRE)
            if now >= self._state_until:
                self._escape_dir = CMD_RIGHT if l >= r else CMD_LEFT
                print(f"[MOTOR] -> REVERSING")
                self._state       = self.REVERSING
                self._state_until = now + REVERSE_TIME_S
            return

        # REVERSING
        if self._state == self.REVERSING:
            self._send_throttle(CMD_REVERSE)
            self._send_steer(CMD_CENTRE)
            if now >= self._state_until:
                print("[MOTOR] -> TURNING")
                self._state       = self.TURNING
                self._state_until = now + TURN_TIME_S
            return

        # TURNING (ANTI-STUCK LOGIC)
        if self._state == self.TURNING:

            if c <= STOP_M:
                self._retry_count += 1
                print(f"[MOTOR] Blocked during TURNING (retry {self._retry_count})")

                if self._retry_count >= self._max_retries:
                    self._retry_count = 0
                    self._escape_dir = CMD_LEFT if self._escape_dir == CMD_RIGHT else CMD_RIGHT
                    print("[MOTOR] Switching direction")

                if self._retry_count >= self._max_retries * 2:
                    print("[MOTOR] STUCK → FAILSAFE STOP")
                    self._state = self.NORMAL
                    self._send_throttle(CMD_STOP)
                    self._send_steer(CMD_CENTRE)
                    return

                self._state       = self.REVERSING
                self._state_until = now + REVERSE_TIME_S
                return

            self._send_throttle(CMD_SLOW)
            self._send_steer(self._escape_dir)

            if now >= self._state_until:
                print("[MOTOR] -> ESCAPING")
                self._state       = self.ESCAPING
                self._state_until = now + ESCAPE_TIME_S
            return

        # ESCAPING
        if self._state == self.ESCAPING:
            self._send_throttle(CMD_SLOW)
            self._send_steer(self._escape_dir)
            if now >= self._state_until:
                print("[MOTOR] -> STRAIGHTENING")
                self._state       = self.STRAIGHTENING
                self._state_until = now + STRAIGHT_TIME_S
            return

        # STRAIGHTENING
        if self._state == self.STRAIGHTENING:
            self._send_throttle(CMD_SLOW)
            self._send_steer(CMD_CENTRE)
            if now >= self._state_until:
                print("[MOTOR] -> NORMAL")
                self._state = self.NORMAL
                self._retry_count = 0
            return

        # NORMAL
        if c <= STOP_M:
            print(f"[MOTOR] Obstacle ahead -> STOPPING")
            self._state       = self.STOPPING
            self._state_until = now + STOP_TIME_S
            return

        self._send_throttle(CMD_FORWARD)
        self._send_steer(CMD_CENTRE)

    def stop(self):
        self._running = False
        self._send(CMD_STOP)
        self._send(CMD_CENTRE)
        print("[MOTOR] Stopped")