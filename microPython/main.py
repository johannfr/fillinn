from OpticalFlow import *
from Sonar import *


class FillinnState:
    def __init__(self):
        self.distance_left = 0
        self.distance_right = 0

        self.adns = OpticalFlow(1)

        self.left_sonar = Sonar(pyb.Pin.board.X1, 'AN') #FIXME
        self.right_sonar = Sonar(pyb.Pin.board.X1, 'AN') #FIXME
        self.left_sonar.enabled(True)
        self.right_sonar.enabled(True)


    def update_state(self):
        self.distance_left = self.left_sonar.getDistance()
        self.distance_right = self.right_sonar.getDistance()
        self.adns.update()


