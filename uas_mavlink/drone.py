
#DO STUFF HEREM WHOEVER IS RESPONNSISBLE FOR MAVLINK COMMECTIONS SHI
from pymavlink import mavutil
class Drone:
    def __init__(self, connection_string, baud=57600):
        self.connection_string = connection_string
        self.baud = baud
        self._conn = None

    def connect(self):
        #connect drone shi
        #reminder for reboot ardu ahn start
        print("[Drone] Connecting...")
        self._conn = mavutil.mavlink_connection(
            self.connection_string,
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
            0,          # confirmation
            1,          # param1: 1=arm
            0, 0, 0, 0, 0, 0  # params 2-7 unused
        )
        self._conn.motors_armed_wait()
        print("[Drone] Armed")

    def disarm(self):
        print("[Drone] Disarming...")
        self._conn.mav.command_long_send(
            self._conn.target_system,
            self._conn.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,          # confirmation
            0,          # param1: 0=disarm
            0, 0, 0, 0, 0, 0  # params 2-7 unused
        )
        self._conn.motors_disarmed_wait()
        print("[Drone] Disarmed")

    def set_mode(self, mode: str):
        mode_mapping = self._conn.mode_mapping()
        if mode not in mode_mapping:
            raise ValueError(f"[Drone] Unknown mode '{mode}'. Available: {list(mode_mapping.keys())}")
        mode_id = mode_mapping[mode]
        self._conn.mav.set_mode_send(
            self._conn.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id
        )
        print(f"[Drone] Mode set to {mode}")
        
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
                "alt": msg.relative_alt / 1000.0  # mm → meters
            }
        return None
    
    def send_body_velocity(self, vx, vy, vz):
    
    ##Velocity relative to drone's current heading.
    ##vx = Forward (+) / Backward (-)
    ##vy = Right (+) / Left (-)
    ##vz = Down (+) / Up (-)
    ##MUST be called repeatedly — stops after ~3s with no command.
    
        self.connection.mav.set_position_target_local_ned_send(
            0,
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
            3527,
            0, 0, 0,       # position (ignored)
            vx, vy, vz,    # velocity (used)
            0, 0, 0,       # acceleration (ignored)
            0, 0           # yaw, yaw_rate (ignored)
        )
        
