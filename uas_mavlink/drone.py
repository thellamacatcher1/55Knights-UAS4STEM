#DO STUFF HEREM WHOEVER IS RESPONNSISBLE FOR MAVLINK COMMECTIONS SHI
from pymavlink import mavutil
import time
import math
import os
import threading
from picamera2 import Picamera2

class Drone:
    def __init__(self, connection_string, baud=57600):
        self._conn_string = connection_string
        self.baud = baud
        self._conn = None
        # this lock wraps every single read/write to self._conn. reason: we've
        # got a background thread (_telem_loop) constantly reading off the
        # serial port, AND the main thread calling arm/set_mode/set_velo_body
        # etc, also on the same port. pyserial does not like two threads
        # calling .read() at the same time, it throws "device reports
        # readiness to read but returned no data" when that happens. learned
        # this the hard way, see the crash log from earlier testing.
        self._port_lock = threading.Lock()
        self._telem_lock = threading.Lock()
        self._telem = {"lat": None, "lon": None, "alt": None, "heading": None}
        self._vel_lock = threading.Lock()
        self._velocity = {"vx": None, "vy": None, "vz": None}
        self._telem_thread = None
        self._telem_stop = threading.Event()

    def connect(self):
        print("[Drone] Connecting...")
        self._conn = mavutil.mavlink_connection(
            self._conn_string,
            baud=self.baud
        )
        print("[Drone] Waiting for heartbeat...")
        with self._port_lock:
            self._conn.wait_heartbeat()
        print(f"[Drone] Connected — system {self._conn.target_system}, component {self._conn.target_component}")

        # kick off the background telemetry thread once we're actually connected.
        # don't want this running before wait_heartbeat succeeds
        self._telem_stop.clear()
        self._telem_thread = threading.Thread(target=self._telem_loop, daemon=True)
        self._telem_thread.start()

    def disconnect(self):
        self._telem_stop.set()
        if self._telem_thread:
            self._telem_thread.join(timeout=2)
        if self._conn:
            self._conn.close()
            self._conn = None
            print("[Drone] Disconnected")

    def _telem_loop(self):
        # runs forever in its own thread, just drains position/velocity
        # messages as they come in and stashes the latest one. the whole
        # point of doing it this way instead of calling recv_match() directly
        # from get_location()/get_velocity() is that recv_match() BLOCKS,
        # sometimes for the full timeout, if the message hasn't shown up yet.
        # that used to stall the whole centering loop in test.py for seconds
        # at a time (found this when testing indoors with no gps fix, every
        # call to get_location() was eating 3 real seconds). now get_location
        # and get_velocity just read whatever's cached here, instant, no wait.
        #
        # timeout here is short (0.1s) on purpose so this loop checks
        # _telem_stop often and doesn't hold _port_lock for long stretches,
        # which would starve other threads (like TrackingController hammering
        # set_velo_body at like 20hz) waiting on the same lock.
        while not self._telem_stop.is_set():
            with self._port_lock:
                msg = self._conn.recv_match(
                    type=['GLOBAL_POSITION_INT', 'LOCAL_POSITION_NED'],
                    blocking=True, timeout=0.1
                )
            if msg is None:
                continue
            mtype = msg.get_type()
            if mtype == 'GLOBAL_POSITION_INT':
                with self._telem_lock:
                    self._telem = {
                        "lat": msg.lat / 1e7,
                        "lon": msg.lon / 1e7,
                        "alt": msg.relative_alt / 1000.0,  # mm to meters
                        "heading": msg.hdg / 100.0,  # centidegrees to degrees
                    }
            elif mtype == 'LOCAL_POSITION_NED':
                with self._vel_lock:
                    self._velocity = {"vx": msg.vx, "vy": msg.vy, "vz": msg.vz}

    def arm(self):
        print("[Drone] Arming...")
        with self._port_lock:
            self._conn.mav.command_long_send(
                self._conn.target_system,
                self._conn.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,
                1,
                0, 0, 0, 0, 0, 0
            )
        # confirmation loop, listens for both heartbeat and statustext.
        # statustext is important here, if arming gets rejected (bad gps,
        # failed prearm check, whatever) the FC usually says why through a
        # statustext message. without listening for it you just get a
        # generic timeout with zero clue what actually went wrong
        start = time.time()
        while time.time() - start < 5:
            with self._port_lock:
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
        with self._port_lock:
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
        # same deal as arm(), listens for statustext too so a rejected mode
        # change actually tells you why instead of just timing out silently
        mode_mapping = self._conn.mode_mapping()
        if mode not in mode_mapping:
            raise ValueError(f"[Drone] Unknown mode '{mode}'. Available: {list(mode_mapping.keys())}")
        mode_id = mode_mapping[mode]
        with self._port_lock:
            self._conn.mav.set_mode_send(
                self._conn.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                mode_id
            )
        start = time.time()
        while time.time() - start < timeout:
            with self._port_lock:
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
        with self._port_lock:
            msg = self._conn.recv_match(type='HEARTBEAT', blocking=True, timeout=3)
        if msg:
            return mavutil.mode_string_v10(msg)
        return None

    def get_armed(self):
        with self._port_lock:
            msg = self._conn.recv_match(type='HEARTBEAT', blocking=True, timeout=3)
        if msg:
            return bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        return None

    def get_location(self):
        # non blocking, just reads whatever the telem thread cached last.
        # returns None if we haven't gotten a GLOBAL_POSITION_INT yet at all
        # (common indoors with no gps fix)
        with self._telem_lock:
            if self._telem["alt"] is None:
                return None
            return dict(self._telem)

    def get_velocity(self):
        with self._vel_lock:
            if self._velocity["vx"] is None:
                return None
            return dict(self._velocity)

    def brake_and_wait(self, threshold=0.3, timeout=10):
        # switches to BRAKE mode and waits til groundspeed drops under the
        # threshold before returning. point of this is so we're not still
        # carrying momentum from whatever flight mode we were just in right
        # when we switch to GUIDED and start centering, if the drone's still
        # sliding sideways the centering math has to fight that on top of
        # everything else. better to come to a real stop first.
        #
        # if BRAKE mode itself fails to engage (rejected, whatever) this just
        # logs it and returns False, doesn't raise. caller decides if that's
        # a big deal or not
        print("[Drone] Switching to BRAKE to stop cleanly...")
        try:
            self.set_mode("BRAKE")
        except RuntimeError as e:
            print(f"[Drone] Could not enter BRAKE: {e}")
            return False

        start = time.time()
        while time.time() - start < timeout:
            vel = self.get_velocity()
            if vel is not None:
                speed = (vel["vx"]**2 + vel["vy"]**2 + vel["vz"]**2) ** 0.5
                print(f"[Drone] BRAKE groundspeed={speed:.2f} m/s")
                if speed < threshold:
                    print("[Drone] Stopped via BRAKE.")
                    return True
            time.sleep(0.1)
        print("[Drone] brake_and_wait timed out — proceeding anyway")
        return False

    def set_pos_body(self, x: float, y: float, z: float):
        with self._port_lock:
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
        # velocity in the BODY frame, relative to wherever the nose is
        # currently pointed. vx is forward positive backward negative, vy is
        # right positive left negative, vz is down positive up negative.
        # this has to get called over and over, ardupilot drops the command
        # and stops moving after about 3 seconds with nothing new coming in,
        # its a safety thing so a lost connection doesn't leave the drone
        # flying off forever
        #
        # yaw rate is sent as an actual 0 here, not marked as ignored in the
        # typemask. reason for that: early on we left yaw rate ignored and
        # ardupilot kept auto rotating the nose to chase whatever direction
        # the velocity vector happened to be pointing at that instant. since
        # our vx/vy changes basically every frame while centering, that meant
        # the drone was constantly spinning in place trying to face "forward"
        # relative to a target direction that kept shifting. explicitly
        # commanding yaw rate = 0 tells it to just hold the current heading
        # and only move translationally, no auto yaw
        with self._port_lock:
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
                    mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
                ),
                0, 0, 0,
                vx, vy, vz,
                0, 0, 0,
                0, 0.0
            )

    def condition_yaw(self, degrees: float, speed: float = 10, direction: int = 1, relative: int = 1):
        with self._port_lock:
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
        with self._port_lock:
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
        with self._port_lock:
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
            with self._port_lock:
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
            with self._port_lock:
                msg = self._conn.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=1)
            if msg is None:
                continue
            speed = (msg.vx**2 + msg.vy**2 + msg.vz**2) ** 0.5
            if speed < 0.1:
                return
        print("[Drone] Warning: position timeout")

    def take_pics(self, max_height: float, height_interval: float, angle_interval: int, auto: bool):
        # walks a dice pattern (center + 4 corners) at increasing heights,
        # spinning in place at each spot to grab training photos from every
        # angle. auto=True flies it for real, auto=False just prompts you to
        # move the rig by hand and hit enter between shots
        SAVE_DIR = "/home/pi/HEF-Models/trainingpics"
        os.makedirs(SAVE_DIR, exist_ok=True)
        count = len(os.listdir(SAVE_DIR))

        OFFSET = 0.61  # about 2 feet in meters
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