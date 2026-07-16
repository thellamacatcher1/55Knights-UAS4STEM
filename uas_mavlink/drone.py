#DO STUFF HEREM WHOEVER IS RESPONNSISBLE FOR MAVLINK COMMECTIONS SHI
from pymavlink import mavutil
import time
import math
import os
from picamera2 import Picamera2

class Drone:
    def __init__(self, connection_string, baud=57600):
        self._conn_string = connection_string
        self.baud = baud
        self._conn = None

    def connect(self):
        print("[Drone] Connecting...")
        self._conn = mavutil.mavlink_connection(
            self._conn_string,
            baud=self.baud
        )
        print("[Drone] Waiting for heartbeat...")
        self._conn.wait_heartbeat()
        print(f"[Drone] Connected — system {self._conn.target_system}, component {self._conn.target_component}")

    def disconnect(self):
        if self._conn:
            self._conn.close()
            self._conn = None
            print("[Drone] Disconnected")

    def arm(self):
        print("[Drone] Arming...")
        self._conn.mav.command_long_send(
            self._conn.target_system,
            self._conn.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1,
            0, 0, 0, 0, 0, 0
        )
        start = time.time()
        while time.time() - start < 5:
            msg = self._conn.recv_match(
                type=['HEARTBEAT', 'STATUSTEXT'],
                blocking=True,
                timeout=1
            )
            if msg is None:
                continue
            if msg.get_type() == 'STATUSTEXT':
                print(f"[Drone] FC: {msg.text.strip()}")
            if msg.get_type() == 'HEARTBEAT':
                if msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
                    print("[Drone] Armed")
                    return
        raise RuntimeError("[Drone] Arming failed — check prearm messages above")

    def disarm(self):
        print("[Drone] Disarming...")
        self._conn.mav.command_long_send(
            self._conn.target_system,
            self._conn.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            0,
            0, 0, 0, 0, 0, 0
        )
        self._conn.motors_disarmed_wait()
        print("[Drone] Disarmed")

    def set_mode(self, mode: str, timeout: float = 5):
        mode_mapping = self._conn.mode_mapping()
        if mode not in mode_mapping:
            raise ValueError(f"[Drone] Unknown mode '{mode}'. Available: {list(mode_mapping.keys())}")
        mode_id = mode_mapping[mode]
        self._conn.mav.set_mode_send(
            self._conn.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id
        )
        start = time.time()
        while time.time() - start < timeout:
            msg = self._conn.recv_match(type=['HEARTBEAT', 'STATUSTEXT'], blocking=True, timeout=1)
            if msg is None:
                continue
            if msg.get_type() == 'STATUSTEXT':
                print(f"[Drone] FC: {msg.text.strip()}")
            if msg.get_type() == 'HEARTBEAT':
                if mavutil.mode_string_v10(msg) == mode:
                    print(f"[Drone] Mode confirmed: {mode}")
                    return
        raise RuntimeError(f"[Drone] Mode change to '{mode}' not confirmed within {timeout}s")

    def get_mode(self):
        msg = self._conn.recv_match(type='HEARTBEAT', blocking=True, timeout=3)
        if msg:
            return mavutil.mode_string_v10(msg)
        return None

    def get_location(self):
        msg = self._conn.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=3)
        if msg:
            return {
                "lat": msg.lat / 1e7,
                "lon": msg.lon / 1e7,
                "alt": msg.relative_alt / 1000.0,
                "heading": msg.hdg / 100.0,
            }
        return None

    def set_pos_body(self, x: float, y: float, z: float):
        self._conn.mav.set_position_target_local_ned_send(
            0,
            self._conn.target_system,
            self._conn.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
            (
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
            ),
            x, y, z,
            0, 0, 0,
            0, 0, 0,
            0, 0
        )

    def set_velo_body(self, vx: float, vy: float, vz: float):
        self._conn.mav.set_position_target_local_ned_send(
            0,
            self._conn.target_system,
            self._conn.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
            (
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
            ),
            0, 0, 0,
            vx, vy, vz,
            0, 0, 0,
            0, 0
        )

    def condition_yaw(self, degrees: float, speed: float = 10, direction: int = 1, relative: int = 1):
        self._conn.mav.command_long_send(
            self._conn.target_system,
            self._conn.target_component,
            mavutil.mavlink.MAV_CMD_CONDITION_YAW,
            0,
            degrees,
            speed,
            direction,
            relative,
            0, 0, 0
        )

    def goto(self, lat: float, lon: float, alt: float, yaw_deg: float = None):
        yaw_rad = math.radians(yaw_deg) if yaw_deg is not None else float('nan')
        self._conn.mav.command_int_send(
            self._conn.target_system,
            self._conn.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            mavutil.mavlink.MAV_CMD_DO_REPOSITION,
            0,
            0,
            -1,
            0,
            0,
            yaw_rad,
            int(lat * 1e7),
            int(lon * 1e7),
            alt
        )
        print(f"[Drone] Going to lat={lat}, lon={lon}, alt={alt}m, yaw={yaw_deg}°")

    def takeoff(self, altitude: float, timeout: float = 30):
        self._conn.mav.command_long_send(
            self._conn.target_system,
            self._conn.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            0, 0, 0, 0, 0, 0,
            altitude
        )
        print(f"[Drone] Taking off to {altitude}m...")
        start = time.time()
        while time.time() - start < timeout:
            msg = self._conn.recv_match(
                type='GLOBAL_POSITION_INT',
                blocking=True,
                timeout=1
            )
            if msg is None:
                continue
            current_alt = msg.relative_alt / 1000.0
            print(f"[Drone] Altitude: {current_alt:.1f}m")
            if current_alt >= altitude * 0.95:
                print(f"[Drone] Reached {altitude}m")
                return
        raise RuntimeError(f"[Drone] Takeoff timeout — never reached {altitude}m")

    def _capture(self, picam2, label: str, count: int) -> int:
        import cv2
        SAVE_DIR = "/home/pi/HEF-Models/trainingpics"
        frame = picam2.capture_array('lores')
        path = os.path.join(SAVE_DIR, f"pic_{count:04d}_{label}.jpg")
        cv2.imwrite(path, frame)
        print(f"[takepics] Saved {path}")
        return count + 1

    def _wait_or_auto(self, msg: str, auto: bool):
        if auto:
            self._wait_for_position()
        else:
            input(f"[takepics] {msg} — press Enter to continue")

    def _wait_for_position(self, timeout=10):
        time.sleep(0.5)
        start = time.time()
        while time.time() - start < timeout:
            msg = self._conn.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=1)
            if msg is None:
                continue
            speed = (msg.vx**2 + msg.vy**2 + msg.vz**2) ** 0.5
            if speed < 0.1:
                return
        print("[Drone] Warning: position timeout")

    def take_pics(self, max_height: float, height_interval: float, angle_interval: int, auto: bool):
        SAVE_DIR = "/home/pi/HEF-Models/trainingpics"
        os.makedirs(SAVE_DIR, exist_ok=True)
        count = len(os.listdir(SAVE_DIR))

        OFFSET = 0.61
        model_w, model_h = 640, 640
        yaw_step = 360.0 / angle_interval

        positions = [
            (0.0,     0.0,    "center"),
            (OFFSET,  -OFFSET, "fwd_left"),
            (OFFSET,   OFFSET, "fwd_right"),
            (-OFFSET, -OFFSET, "back_left"),
            (-OFFSET,  OFFSET, "back_right"),
        ]

        picam2 = Picamera2()
        main_cfg = {'size': (1088, 1088), 'format': 'XRGB8888'}
        lores_cfg = {'size': (model_w, model_h), 'format': 'RGB888'}
        config_cam = picam2.create_preview_configuration(main_cfg, lores=lores_cfg)
        picam2.configure(config_cam)
        picam2.start()
        picam2.set_controls({"ScalerCrop": (184, 0, 1088, 1088)})
        time.sleep(1)

        try:
            height = height_interval
            while height <= max_height:
                print(f"[takepics] Climbing to {height}m...")
                if auto:
                    if height == height_interval:
                        self.takeoff(height)
                    else:
                        self.set_pos_body(0, 0, -height_interval)

                self._wait_or_auto(f"At {height}m", auto)

                for (px, py, pos_name) in positions:
                    print(f"[takepics] Moving to {pos_name}...")

                    if px != 0 or py != 0:
                        if auto: self.set_pos_body(px, py, 0)
                        self._wait_or_auto(f"At {pos_name}", auto)

                    for r in range(angle_interval):
                        print(f"[takepics] Rotation {r+1}/{angle_interval}")
                        if auto: self.condition_yaw(yaw_step, speed=20, direction=1, relative=1)
                        self._wait_or_auto(f"Rotation {r+1}", auto)
                        label = f"h{height:.0f}_{pos_name}_r{r}"
                        count = self._capture(picam2, label, count)

                    if px != 0 or py != 0:
                        if auto: self.set_pos_body(-px, -py, 0)
                        self._wait_or_auto("Returning to center", auto)

                height = round(height + height_interval, 2)
            print(f"[takepics] Done — {count} pics saved to {SAVE_DIR}")

        finally:
            picam2.stop()
            if auto:
                self.set_mode("LAND")
                print("[takepics] Landing")
            else:
                print("[takepics] Done — Manual")