import threading
import time
import config


class TrackingController:
    CENTER_DEADZONE = 20  # pixels. shared with test.py so both sides agree on what "centered" means

    MIN_SPEED = 0.0    # m/s floor, no minimum right now, small offsets get small commands
    MAX_SPEED = 1.0     # m/s cap, kept low on purpose for early testing, way under the 4mph safety limit
    KP = 0.006           # proportional gain, still a rough guess, tune this against real flight data

    # once alt drops below this while centered, ready_for_final_land() starts
    # returning true and test.py hands off to LAND mode for the actual touchdown.
    # ardupilot's own LAND does a controlled descent better than anything we'd
    # write by hand here, so this class just gets the drone low and steady,
    # then gets out of the way
    FINAL_LAND_ALT = 0.5  # meters

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

    def ready_for_final_land(self):
        # true once we're both centered on target AND low enough that
        # handing off to LAND mode for the last bit of descent makes sense.
        # test.py polls this during the "descending" phase to decide when to
        # stop commanding velocity ourselves and just let ardupilot land
        if not self.drone:
            return False
        loc = self.drone.get_location()
        if loc is None:
            return False

        offsets = self.detector.get_offsets()
        offset = offsets.get(self.target_class)
        if offset is None:
            return False

        dx, dy = offset
        centered = abs(dx) <= self.CENTER_DEADZONE and abs(dy) <= self.CENTER_DEADZONE
        return centered and loc["alt"] <= self.FINAL_LAND_ALT

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

        vx = 0.0  # forward positive, backward negative
        vy = 0.0  # right positive, left negative
        vz = 0.0  # down positive, up negative

        if target_visible:
            # x tracking, image left/right maps to body right/left
            if abs(offset_x) > self.CENTER_DEADZONE:
                raw = offset_x * self.KP
                speed = max(self.MIN_SPEED, min(self.MAX_SPEED, abs(raw)))
                vy = speed if raw > 0 else -speed

            # y tracking. bench tested and confirmed with the sign flipped
            # from the naive assumption. dy positive in our pipeline needs
            # the negative sign here to actually move the drone forward
            # toward the target, not away from it
            if abs(offset_y) > self.CENTER_DEADZONE:
                raw = -offset_y * self.KP
                speed = max(self.MIN_SPEED, min(self.MAX_SPEED, abs(raw)))
                vx = speed if raw > 0 else -speed

            # descend once we're centered and have a real altitude reading.
            # proportional term capped at lower_speed so it eases off as we
            # get close to target_alt instead of slamming straight down
            centered = abs(offset_x) <= self.CENTER_DEADZONE and abs(offset_y) <= self.CENTER_DEADZONE
            if centered and self.drone:
                loc = self.drone.get_location()
                if loc is not None:
                    alt_error = loc["alt"] - self.target_alt
                    if alt_error > 0.05:
                        vz = min(self.lower_speed, 0.5 * alt_error)

        self.drone.set_velo_body(vx, vy, vz)

        plain = self._describe(vx, vy, vz, target_visible)
        print(f"Velocity cmd: vx={vx:.2f}, vy={vy:.2f}, vz={vz:.2f}  -> {plain}")

        if self.detector:
            self.detector.set_status_text(plain)

    def _describe(self, vx, vy, vz, target_visible=True):
        # plain english version of the velocity command, gets written into
        # the corner of the preview overlay by the detector, and printed to
        # terminal. purely for reading the logs and eyeballing the window,
        # doesn't affect any actual control logic
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