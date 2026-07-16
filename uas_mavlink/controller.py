import threading
import time
import config


class TrackingController:
    CENTER_DEADZONE = 50

    MIN_SPEED = 0.0
    MAX_SPEED = 1.0
    KP = 0.006

    def __init__(self, detector, drone=None, target_class="person", lower_speed=0.01):
        self.detector = detector
        self.drone = drone
        self.target_class = target_class
        self.lower_speed = lower_speed
        self.lower_distance = 0
        self.is_tracking = False
        self.frame_count = 0
        self._tracking_thread = None
        self._stop_event = threading.Event()
        self.target_alt = 1.0

    def set_target_class(self, target_class):
        self.target_class = target_class

    def set_lower_speed(self, speed):
        self.lower_speed = speed

    def start_tracking(self):
        if not self.is_tracking:
            self.is_tracking = True
            self._stop_event.clear()
            self._tracking_thread = threading.Thread(target=self._tracking_loop, daemon=True)
            self._tracking_thread.start()
            print("track started")

    def stop_tracking(self):
        if self.is_tracking:
            self._stop_event.set()
            self.is_tracking = False
            if self._tracking_thread:
                self._tracking_thread.join()
            print("track stopped")

    def _tracking_loop(self):
        last_time = time.time()

        try:
            while not self._stop_event.is_set():
                offsets = self.detector.get_offsets()
                offset = offsets.get(self.target_class)

                current_time = time.time()
                dt = current_time - last_time
                last_time = current_time

                if offset is not None:
                    offset_x, offset_y = offset
                    if dt > 0:
                        self._update_lowering(dt)
                    self._control_drone(offset_x, offset_y, target_visible=True)
                    target_x = offset_x + config.VIDEO_W // 2
                    target_y = offset_y + config.VIDEO_H // 2
                    print(f"Frame {self.frame_count}: Target '{self.target_class}' @ ({target_x}, {target_y}), "
                          f"offset dx={offset_x:+d}, dy={offset_y:+d}, lowering={self.lower_distance:.2f}")
                else:
                    self._control_drone(0, 0, target_visible=False)
                    print(f"Frame {self.frame_count}: No target, lowering={self.lower_distance:.2f}")

                self.frame_count += 1
                time.sleep(0.05)

        except Exception as e:
            print(f"Error in tracking loop: {e}")

    def _update_lowering(self, dt):
        self.lower_distance += self.lower_speed * dt

    def _control_drone(self, offset_x, offset_y, target_visible=True):
        if not self.drone:
            return

        vx = 0.0  # forward(+)/backward(-)
        vy = 0.0  # right(+)/left(-)
        vz = 0.0  # down(+)/up(-) — not active yet, see Future Altitude Hold below

        if target_visible:
            # ---------- X Tracking (image left/right -> body right/left) ----------
            if abs(offset_x) > self.CENTER_DEADZONE:
                raw = offset_x * self.KP
                speed = max(self.MIN_SPEED, min(self.MAX_SPEED, abs(raw)))
                vy = speed if raw > 0 else -speed

            # ---------- Y Tracking ----------
            # Bench-confirmed: sign flipped from the naive "dy positive = top =
            # front" assumption — actual camera mount/pipeline needed this
            # negation to match physical forward/backward correctly.
            if abs(offset_y) > self.CENTER_DEADZONE:
                raw = -offset_y * self.KP
                speed = max(self.MIN_SPEED, min(self.MAX_SPEED, abs(raw)))
                vx = speed if raw > 0 else -speed

        # ---------- Future Altitude Hold ----------
        #
        # target_altitude = 1.0
        # current_alt = self.drone.get_altitude()
        #
        # if current_alt is not None:
        #     if current_alt > target_altitude + 0.1:
        #         vz = 0.20
        #     elif current_alt < target_altitude - 0.1:
        #         vz = -0.20

        self.drone.set_velo_body(vx, vy, vz)

        # _describe reads vx/vy AFTER they're finalized above, so the printed/
        # drawn text always matches whatever was actually just commanded —
        # no separate sync needed if a sign ever changes again later.
        plain = self._describe(vx, vy, vz, target_visible)
        print(f"Velocity cmd: vx={vx:.2f}, vy={vy:.2f}, vz={vz:.2f}  -> {plain}")

        if self.detector:
            self.detector.set_status_text(plain)

    def _describe(self, vx, vy, vz, target_visible=True):
        if not target_visible:
            return "NO TARGET"

        ns = "Forward" if vx > 1e-6 else ("Back" if vx < -1e-6 else "")
        ew = "Right" if vy > 1e-6 else ("Left" if vy < -1e-6 else "")
        base = f"{ns}{ew}" if (ns or ew) else "Hold"

        if vz > 1e-6:
            base += " + Descend"
        elif vz < -1e-6:
            base += " + Climb"

        return base

    def get_tracking_status(self):
        return {
            "is_tracking": self.is_tracking,
            "target_class": self.target_class,
            "lower_distance": self.lower_distance,
            "frame_count": self.frame_count
        }

    def get_target_detections(self, detections):
        return [d for d in detections if d[0] == self.target_class]