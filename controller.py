import threading
import time
import config


class TrackingController:
    def __init__(self, detector, drone=None, target_class="person", lower_speed=0.01):
        self.detector = detector
        self.drone = drone
        self.target_class = target_class
        self.lower_speed = lower_speed
        self.lower_distance = 0
        self.is_tracking = False
        self.target_center_x = 0
        self.target_center_y = 0
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
                detections = self.get_target_detections(self.detector.get_detections())

                offset_x, offset_y, target_x, target_y = self._center_target(detections)

                current_time = time.time()
                dt = current_time - last_time
                last_time = current_time

                if dt > 0:
                    self._update_lowering(dt)

                self._control_drone(offset_x, offset_y)

                if detections:
                    print(f"Frame {self.frame_count}: Target '{self.target_class}' @ ({target_x}, {target_y}), "
                          f"offset dx={offset_x:+d}, dy={offset_y:+d}, lowering={self.lower_distance:.2f}")
                else:
                    print(f"Frame {self.frame_count}: No target, lowering={self.lower_distance:.2f}")

                self.frame_count += 1
                time.sleep(0.05)

        except Exception as e:
            print(f"Error in tracking loop: {e}")

    def _center_target(self, detections):
        if not detections:
            return 0, 0, 0, 0  # No target

        best_detection = max(detections, key=lambda d: d[2])
        class_name, bbox, score = best_detection
        x0, y0, x1, y1 = bbox
        center_x = (x0 + x1) // 2
        center_y = (y0 + y1) // 2

        # center
        frame_center_x = config.VIDEO_W // 2
        frame_center_y = config.VIDEO_H // 2
        offset_x = center_x - frame_center_x
        offset_y = center_y - frame_center_y

        return offset_x, offset_y, center_x, center_y

    def _update_lowering(self, dt):
        self.lower_distance += self.lower_speed * dt

    def _control_drone(self, offset_x, offset_y):
        if not self.drone:
            return

        # Pixel deadzone
        DEADZONE = 20

        # Speeds (m/s)
        STRAFE_SPEED = 0.25
        VERTICAL_SPEED = 0.20

        vx = 0.0      # forward(+)/backward(-)
        vy = 0.0      # right(+)/left(-)
        vz = 0.0      # down(+)/up(-) — not active yet, see Future Altitude Hold below

        # ---------- X Tracking (image left/right -> body right/left) ----------
        if offset_x > DEADZONE:
            vy = STRAFE_SPEED          # target right of center -> move right
        elif offset_x < -DEADZONE:
            vy = -STRAFE_SPEED         # target left of center -> move left

        # ---------- Y Tracking (image up/down -> body forward/back, nadir cam) ----------
        # Down-facing camera: image-top = drone nose (forward), image-bottom = behind.
        # Target below center (offset_y > 0) means it's physically BEHIND the drone,
        # so we move backward (vx negative) to close the gap.
        # NOTE: verify this sign on a bench test first — if the cam mount is rotated
        # relative to this assumption, flip both branches below.
        if offset_y > DEADZONE:
            vx = -STRAFE_SPEED         # target below center (behind) -> move backward
        elif offset_y < -DEADZONE:
            vx = STRAFE_SPEED          # target above center (ahead) -> move forward

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

        print(
            f"Velocity cmd: vx={vx:.2f}, "
            f"vy={vy:.2f}, vz={vz:.2f}"
        )

    def get_tracking_status(self):
        return {
            "is_tracking": self.is_tracking,
            "target_class": self.target_class,
            "lower_distance": self.lower_distance,
            "frame_count": self.frame_count
        }

    def get_target_detections(self, detections):
        target_detections = [d for d in detections if d[0] == self.target_class]
        return target_detections
