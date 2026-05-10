
#DO STUFF HEREM WHOEVER IS RESPONNSISBLE FOR MAVLINK COMMECTIONS SHI
from pymavlink import mavutil
import time
import math
class Drone:
    def __init__(self, connection_string, baud=57600):
        self._conn_string = connection_string
        self.baud = baud
        self._conn = None

    def connect(self):
        #connect drone shi
        #reminder for reboot ardu ahn start
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
    
 
        
    def set_pos(self, mode: str, x: float, y: float, z: float):
    
        """
        mode: POS or VELO, all relative to drone heading
        x: forward(+) / backward(-)
        y: right(+) / left(-)
        z: down(+) / up(-)
        For yaw use condition_yaw() separately.
        """
    
        if mode == "POS":
            type_mask = (
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
            )
            px, py, pz = x, y, z
            vx, vy, vz = 0, 0, 0
        elif mode == "VELO":
            type_mask = (
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
            )
            px, py, pz = 0, 0, 0
            vx, vy, vz = x, y, z
        else:
            raise ValueError(f"[Drone] Unknown mode '{mode}'. Use 'POS' or 'VELO'")

        self._conn.mav.set_position_target_local_ned_send(
            0,
            self._conn.target_system,
            self._conn.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
            type_mask,
            px, py, pz,
            vx, vy, vz,
            0, 0, 0,
            0, 0
        )
    
    def condition_yaw(self, degrees: float, speed: float = 10, direction: int = 1, relative: int = 1):
    
        # Rotate the drone's yaw.
        # degrees:   angle in degrees
        # speed:     rotation speed in deg/s (default 10)
        # direction: 1=CW, -1=CCW, 0=shortest path (only works if relative=0)
        # relative:  1=relative to current heading, 0=absolute heading
    
        self._conn.mav.command_long_send(
            self._conn.target_system,
            self._conn.target_component,
            mavutil.mavlink.MAV_CMD_CONDITION_YAW,
            0,           # confirmation
            degrees,     # param1: angle in degrees
            speed,       # param2: deg/s
            direction,   # param3: 1=CW, -1=CCW, 0=shortest
            relative,    # param4: 1=relative, 0=absolute
            0, 0, 0      # params 5-7 unused
        )
    
    def goto(self, lat: float, lon: float, alt: float, yaw_deg: float = None):
        """
        Fly to GPS coordinate using MAV_CMD_DO_REPOSITION.
        lat, lon:  decimal degrees
        alt:       meters relative to home
        yaw_deg:   absolute yaw in degrees (0=North). None = keep current heading.
        """
        yaw_rad = math.radians(yaw_deg) if yaw_deg is not None else float('nan')

        self._conn.mav.command_int_send(
            self._conn.target_system,
            self._conn.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            mavutil.mavlink.MAV_CMD_DO_REPOSITION,
            0,                  # current (unused)
            0,                  # autocontinue (unused)
            -1,                 # param1: speed -1 = default
            0,                  # param2: bitmask flags
            0,                  # param3: loiter radius (planes only)
            yaw_rad,            # param4: yaw in radians, nan = keep heading
            int(lat * 1e7),     # param5: latitude scaled integer
            int(lon * 1e7),     # param6: longitude scaled integer
            alt                 # param7: altitude meters
        )
        print(f"[Drone] Going to lat={lat}, lon={lon}, alt={alt}m, yaw={yaw_deg}°")
          
