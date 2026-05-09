class Controller:
    def __init__(self, drone, detector):
        self.drone = drone
        self.detector = detector

    def run_loop(self):
        #trakcign shi
        raise NotImplementedError