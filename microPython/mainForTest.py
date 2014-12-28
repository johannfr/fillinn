# main.py -- put your code here!
from Sonar import *

DIST_TO_BUTTON = 0.2                    # Value in meters
LEFT_SONAR_PIN = pyb.Pin.board.X1       # Analog pin for the left sonar

def buttonIsHere(sonar):
    distance = sonar.getDistance()
    print('Dist: ', distance)

    if distance < DIST_TO_BUTTON:
        print('Button Found!')
        return False
    return True

def setSpeedValue():
    # TODO: Send the commanded speed to the Arduino
    pass


if __name__ == "__main__":
    buttonPressed = False
    sonar = Sonar(LEFT_SONAR_PIN, 'AN')

    while  True:
        if not buttonPressed:
            if buttonIsHere(sonar):
                # push the button
                pass
            # keep driving

        if buttonPressed:
            # look for the can
            pass

        setSpeedValue()