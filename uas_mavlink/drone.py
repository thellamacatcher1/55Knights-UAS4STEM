
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
        raise NotImplementedError

    def disconnect(self):
        if self._conn:
            self._conn.close()
            self._conn = None

    def arm(self):
        raise NotImplementedError

    def disarm(self):
        raise NotImplementedError

    def set_mode(self, mode: str):
        raise NotImplementedError