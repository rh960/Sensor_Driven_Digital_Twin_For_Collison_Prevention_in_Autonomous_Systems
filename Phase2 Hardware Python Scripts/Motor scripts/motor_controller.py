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

# ── Distance zones (metres) ───────────────────────────────────
ZONE_CLEAR    = 1.5   # front clear — full speed
ZONE_CAUTION  = 1.0   # slow down + gentle steer
ZONE_STEER    = 0.8   # steer around, no stop
ZONE_STOP     = 0.6   # must reverse

SIDE_WALL_M   = 1.2   # steer away from side walls within this
SIDE_OPEN_M   = 0.5   # side must be at least this clear to bypass

# ── Timings ───────────────────────────────────────────────────
REVERSE_TIME_S  = 2.5
STOP_BRAKE_S    = 0.3
STUCK_STEER_S   = 4.0   # declare stuck after steering this long with no clear

# ── Anti-stuck strategy (from your original design) ───────────
MAX_RETRIES     = 3     # attempts per direction before flipping

LOOP_HZ         = 20


class MotorController:

    NORMAL    = "NORMAL"
    BRAKING   = "BRAKING"
    REVERSING = "REVERSING"
    TURNING   = "TURNING"
    ESCAPING  = "ESCAPING"

    def __init__(self):
        self._sock              = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._paused            = False
        self._state             = self.NORMAL
        self._state_until       = 0.0

        # Anti-stuck: counts full reverse attempts per direction
        self._retry_count       = 0       # increments each time TURNING sees blocked
        self._retry_counted     = False   # ensures count only increments once per entry
        self._escape_dir        = CMD_RIGHT
        self._total_flips       = 0

        # Stuck steering detection
        self._is_steering       = False
        self._steering_since    = 0.0

        # Escape confirmation
        self._escape_start      = 0.0

        self._log_timer         = 0.0
        self._lock              = threading.Lock()
        self._level             = "SAFE"
        self._fl                = None
        self._fc                = None
        self._fr                = None
        self._running           = True

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
                    with self._lock:
                        self._paused        = True
                        self._state         = self.NORMAL
                        self._retry_count   = 0
                        self._retry_counted = False
                        self._total_flips   = 0
                        self._is_steering   = False
                    self._send(CMD_STOP)
                    self._send(CMD_CENTRE)
                    print("[MOTOR] PAUSED")
                elif cmd == 'R':
                    with self._lock:
                        self._paused        = False
                        self._state         = self.NORMAL
                        self._retry_count   = 0
                        self._retry_counted = False
                        self._total_flips   = 0
                        self._is_steering   = False
                    print("[MOTOR] RESUMED")
            except socket.timeout:
                continue
            except Exception as e:
                print(f"[MOTOR] Listener: {e}")

    def _send(self, cmd: bytes):
        try:
            self._sock.sendto(cmd, (ARDUINO_IP, ARDUINO_PORT))
        except Exception as e:
            print(f"[MOTOR] UDP error: {e}")

    def _send_throttle(self, cmd: bytes):
        self._send(cmd)

    def _send_steer(self, cmd: bytes):
        self._send(cmd)

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
            with self._lock:
                paused = self._paused
                fl     = self._fl
                fc     = self._fc
                fr     = self._fr
            if not paused:
                self._process(fl, fc, fr)
            elapsed = time.time() - t0
            sleep = interval - elapsed
            if sleep > 0:
                time.sleep(sleep)

    def _open_side(self, l, r):
        return CMD_LEFT if l >= r else CMD_RIGHT

    def _process(self, fl, fc, fr):
        now = time.time()
        l = fl if fl is not None else 9.0
        c = fc if fc is not None else 9.0
        r = fr if fr is not None else 9.0

        if now - self._log_timer > 0.5:
            self._log_timer = now
            print(f"[MOTOR] {self._state} | L={l:.2f} C={c:.2f} R={r:.2f} "
                  f"retry={self._retry_count} flips={self._total_flips}")

        # ══════════════════════════════════════════════════════
        # BRAKING — brief stop before reversing
        # ══════════════════════════════════════════════════════
        if self._state == self.BRAKING:
            self._send_throttle(CMD_STOP)
            self._send_steer(CMD_CENTRE)
            if now >= self._state_until:
                # Pick escape direction toward more open side
                self._escape_dir  = self._open_side(l, r)
                self._retry_counted = False
                print(f"[MOTOR] -> REVERSING {REVERSE_TIME_S}s "
                      f"dir={'R' if self._escape_dir == CMD_RIGHT else 'L'}")
                self._state       = self.REVERSING
                self._state_until = now + REVERSE_TIME_S
            return

        # ══════════════════════════════════════════════════════
        # REVERSING — full duration always
        # ══════════════════════════════════════════════════════
        if self._state == self.REVERSING:
            self._send_throttle(CMD_REVERSE)
            self._send_steer(CMD_CENTRE)
            if now >= self._state_until:
                # Re-read direction after full reverse
                self._escape_dir    = self._open_side(l, r)
                self._retry_counted = False
                print(f"[MOTOR] -> TURNING "
                      f"dir={'R' if self._escape_dir == CMD_RIGHT else 'L'}")
                self._state = self.TURNING
            return

        # ══════════════════════════════════════════════════════
        # TURNING — your original anti-stuck strategy applied here
        #
        # Every tick while front is blocked:
        #   _retry_count increments ONCE per entry (not per tick)
        #   After MAX_RETRIES → flip direction, reset count
        #
        # Car turns until front clears (sensor-driven, no timer)
        # ══════════════════════════════════════════════════════
        if self._state == self.TURNING:

            if c <= ZONE_STOP:
                # Front still blocked — apply anti-stuck strategy
                # Count once per TURNING entry, not every 50ms tick
                if not self._retry_counted:
                    self._retry_counted = True
                    self._retry_count  += 1
                    print(f"[MOTOR] TURNING blocked "
                          f"(attempt {self._retry_count}/{MAX_RETRIES}) "
                          f"dir={'R' if self._escape_dir == CMD_RIGHT else 'L'}")

                    # After MAX_RETRIES flip direction — exactly your strategy
                    if self._retry_count >= MAX_RETRIES:
                        self._retry_count = 0
                        self._escape_dir  = CMD_LEFT if self._escape_dir == CMD_RIGHT else CMD_RIGHT
                        self._total_flips += 1
                        print(f"[MOTOR] Anti-stuck FLIP -> "
                              f"{'L' if self._escape_dir == CMD_LEFT else 'R'} "
                              f"(#{self._total_flips})")

                # Keep turning toward escape direction
                side_in_escape = l if self._escape_dir == CMD_LEFT else r
                if side_in_escape < 0.20:
                    # Escape side wall too close — steer centre briefly
                    self._send_throttle(CMD_SLOW)
                    self._send_steer(CMD_CENTRE)
                else:
                    self._send_throttle(CMD_SLOW)
                    self._send_steer(self._escape_dir)
                return

            # Front is clearing — keep turning until fully clear
            if c <= ZONE_CAUTION:
                side_in_escape = l if self._escape_dir == CMD_LEFT else r
                if side_in_escape < 0.20:
                    self._send_throttle(CMD_SLOW)
                    self._send_steer(CMD_CENTRE)
                else:
                    self._send_throttle(CMD_SLOW)
                    self._send_steer(self._escape_dir)
                return

            # Front fully clear — escape
            print(f"[MOTOR] TURNING: front clear C={c:.2f}m -> ESCAPING")
            self._retry_counted = False
            self._escape_start  = now
            self._state         = self.ESCAPING
            return

        # ══════════════════════════════════════════════════════
        # ESCAPING — forward burst to confirm clear path
        # New obstacle → back to BRAKING
        # Clear for 1.0s → NORMAL, reset all counters
        # ══════════════════════════════════════════════════════
        if self._state == self.ESCAPING:
            time_in_escape = now - self._escape_start

            if c <= ZONE_STOP:
                print(f"[MOTOR] ESCAPING: new obstacle C={c:.2f}m -> BRAKING")
                self._state       = self.BRAKING
                self._state_until = now + STOP_BRAKE_S
                self._send_throttle(CMD_STOP)
                self._send_steer(CMD_CENTRE)
                return

            # Side wall check
            side_in_escape = l if self._escape_dir == CMD_LEFT else r
            if side_in_escape < 0.25:
                self._send_throttle(CMD_SLOW)
                self._send_steer(CMD_CENTRE)
            else:
                self._send_throttle(CMD_SLOW)
                self._send_steer(self._escape_dir)

            # Confirmed clear for 1.0s — return to normal
            if time_in_escape >= 1.0:
                print(f"[MOTOR] ESCAPING: confirmed clear -> NORMAL")
                self._state         = self.NORMAL
                self._retry_count   = 0
                self._total_flips   = 0
                self._retry_counted = False
                self._is_steering   = False
            return

        # ══════════════════════════════════════════════════════
        # NORMAL — TurtleBot reactive control
        # Continuous every tick, no timers
        # ══════════════════════════════════════════════════════

        # Side wall steering
        if l < SIDE_WALL_M and r < SIDE_WALL_M:
            side_steer = CMD_RIGHT if l < r else CMD_LEFT
        elif l < SIDE_WALL_M:
            side_steer = CMD_RIGHT
        elif r < SIDE_WALL_M:
            side_steer = CMD_LEFT
        else:
            side_steer = CMD_CENTRE

        # Zone 1: must reverse
        if c <= ZONE_STOP:
            left_open  = l >= SIDE_OPEN_M
            right_open = r >= SIDE_OPEN_M
            if not left_open and not right_open:
                # All blocked — reverse
                print(f"[MOTOR] ALL BLOCKED C={c:.2f} L={l:.2f} R={r:.2f} -> BRAKING")
                self._is_steering   = False
                self._steering_since = 0.0
                self._state         = self.BRAKING
                self._state_until   = now + STOP_BRAKE_S
                self._send_throttle(CMD_STOP)
                self._send_steer(CMD_CENTRE)
            else:
                # One side open — bypass
                steer = CMD_LEFT if (left_open and l >= r) else CMD_RIGHT
                self._send_throttle(CMD_SLOW)
                self._send_steer(steer)
                if not self._is_steering:
                    self._is_steering    = True
                    self._steering_since = now

        # Zone 2: steer around
        elif c <= ZONE_STEER:
            left_open  = l >= SIDE_OPEN_M
            right_open = r >= SIDE_OPEN_M
            if left_open or right_open:
                steer = CMD_LEFT if (left_open and l >= r) else CMD_RIGHT
                if steer == CMD_LEFT and l < 0.30:
                    steer = CMD_RIGHT if right_open else CMD_CENTRE
                elif steer == CMD_RIGHT and r < 0.30:
                    steer = CMD_LEFT if left_open else CMD_CENTRE
                self._send_throttle(CMD_SLOW)
                self._send_steer(steer)
            else:
                self._send_throttle(CMD_SLOW)
                self._send_steer(self._open_side(l, r))

            if not self._is_steering:
                self._is_steering    = True
                self._steering_since = now
            elif now - self._steering_since > STUCK_STEER_S:
                # Been steering too long without clearing — stuck
                print(f"[MOTOR] STUCK steering {now - self._steering_since:.1f}s -> BRAKING")
                self._is_steering    = False
                self._steering_since = 0.0
                self._state          = self.BRAKING
                self._state_until    = now + STOP_BRAKE_S
                self._send_throttle(CMD_STOP)
                self._send_steer(CMD_CENTRE)
                return

        # Zone 3: caution
        elif c <= ZONE_CAUTION:
            self._is_steering    = False
            self._steering_since = 0.0
            if side_steer == CMD_LEFT and l < 0.35:
                side_steer = CMD_RIGHT if r > l else CMD_CENTRE
            elif side_steer == CMD_RIGHT and r < 0.35:
                side_steer = CMD_LEFT if l > r else CMD_CENTRE
            self._send_throttle(CMD_SLOW)
            self._send_steer(side_steer)

        # Zone 4: all clear
        else:
            self._is_steering    = False
            self._steering_since = 0.0
            self._send_throttle(CMD_FORWARD)
            self._send_steer(side_steer)

    def stop(self):
        self._running = False
        self._send(CMD_STOP)
        self._send(CMD_CENTRE)
        print("[MOTOR] Stopped")