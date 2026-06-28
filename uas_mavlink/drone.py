import time
from pymavlink import mavutil

class Drone:
    def __init__(self, connection_string, baudrate=57600):
        self.connection_string = connection_string
        self.baudrate = baudrate
        self.master = None

    def connect(self):
        print(f"Connecting to drone at {self.connection_string}")
        self.master = mavutil.mavlink_connection(self.connection_string, baud=self.baudrate)
        self.master.wait_heartbeat()
        print("Drone connected")

    def disconnect(self):
        if self.master:
            self.master.close()
            print("Drone disconnected")

    def arm(self):
        if self.master:
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,
                1, 0, 0, 0, 0, 0, 0
            )
            print("Drone armed")

    def disarm(self):
        if self.master:
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,
                0, 0, 0, 0, 0, 0, 0
            )
            print("Drone disarmed")

    def send_position(self, x, y, z):
        if self.master:
            self.master.mav.set_position_target_local_ned_send(
                0,
                self.master.target_system,
                self.master.target_component,
                1,
                0b0000111111000111,
                x, y, z,
                0, 0, 0,
                0, 0, 0,
                0, 0
            )