import pyb

class Sonar:

    def __init__(self, inputPin, inputType='AN'):
        self.type = inputType
        self.inputPin = inputPin

        self.setSonar(inputType)

    def setSonar(self, inputType):
        if self.type == 'AN':
            sonarPin = pyb.Pin(self.inputPin, pyb.Pin.IN)
            self.adc = pyb.ADC(sonarPin)                     # https://micropython.org/doc/module/pyb/ADC
        elif self.type == 'PW':
            # TODO
            pass

    def getDistance(self):
        if self.type == 'AN':
            rawValue = self.adc.read()
            # http://playground.arduino.cc/Main/MaxSonar
            # Scale factor is (Vcc/512) per inch. A 3.3V supply yields ~6.5mV/in
            # MicroPython analog pin goes from 0 to 4095, so the value has to be divided by 8 to get the actual inches
            # inches*2.45 => cm
            retValue = rawValue*3.3/512.0/8.0*2.54
        elif self.type == 'PW':
            # TODO
            pass

        return retValue



if __name__ == "__main__":
    test = Sonar(pyb.Pin.board.X1, 'AN')
    
    led = pyb.LED(2)
    while True:
        dist = test.getDistance()
        print('Distance in meters:', dist)
        led.toggle()
        pyb.delay(500)