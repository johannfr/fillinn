import pyb

ADNS3080_SQUAL = 0x05
ADNS3080_FRAME_PERIOD_MAX_BOUND_LOWER = 0x19
ADNS3080_FRAME_PERIOD_MAX_BOUND_UPPER = 0x1a
ADNS3080_FRAME_PERIOD_MIN_BOUND_LOWER = 0x1b
ADNS3080_FRAME_PERIOD_MIN_BOUND_UPPER = 0x1c

# ADNS3080 hardware config
ADNS3080_PIXELS_X = 30
ADNS3080_PIXELS_Y = 30
ADNS3080_CLOCK_SPEED =24000000

ADNS3080_FRAME_CAPTURE = 0x13

ADNS3080_PRODUCT_ID = 0x00

# SPI serial pins
AP_SPI_DATAOUT =      'Y8'        #MOSI
AP_SPI_DATAIN  =      'Y7'        #MISO
AP_SPI_CLOCK   =      'Y6'        #SCK
ADNS3080_CHIP_SELECT = 'Y5'       #SS
ADNS3080_RESET =       0          #RESET

class OpticalFlow:
    def __init__(self, cs_pin='Y5'):
        self.init(cs_pin)

    def do_test(self):
        print('Start Test')
        
        surface_quality = self.read_register(ADNS3080_SQUAL)
        print('surface_quality: ', surface_quality)

        print('End Test')

    def init(self, cs_pin):
        retry = 0

        #---- Not sure if all of this is needed ----#
        pyb.Pin(AP_SPI_DATAOUT, pyb.Pin.OUT_PP)
        pyb.Pin(AP_SPI_DATAIN, pyb.Pin.IN)
        pyb.Pin(AP_SPI_CLOCK, pyb.Pin.OUT_PP)
        self._cs_pin = pyb.Pin(cs_pin, pyb.Pin.OUT_PP)

        if( ADNS3080_RESET != 0):
            self._reset_pin = pyb.Pin(ADNS3080_RESET, pyb.Pin.OUT_PP)
        #-------------------------------------------#

        self._cs_pin.high()                     # disable device (Chip select is active low)

        # reset the device
        self.reset()

        # start the SPI library
        self.spi = pyb.SPI(2, pyb.SPI.MASTER, baudrate=ADNS3080_CLOCK_SPEED, polarity=1, phase=1, firstbit=pyb.SPI.MSB)

        # check the sensor is functioning
        while( retry < 3 ):
            function = self.read_register(ADNS3080_PRODUCT_ID)
            if( function == 0x17 ):
                #return true;
                print('The OpticalFlow sensor is functioning')
            retry += 1
        #return false;      # TODO: delete this debug text
        print('The OpticalFlow sensor is NOT functioning')

    # reset sensor by holding a pin high (or is it low?) for 10us.
    def reset(self):
        # return immediately if the reset pin is not defined
        if( ADNS3080_RESET == 0):
            return

        self._reset_pin.high()                  # reset sensor
        pyb.delay(10)
        self._reset_pin.low()                   # return sensor to normal
        
    # get_pixel_data - captures an image from the sensor and stores it to the pixe_data array
    def print_pixel_data(self, serPort):            # TODO: send data to serPort do not print the data here
        isFirstPixel = True

        # write to frame capture register to force capture of frame
        self.write_register(ADNS3080_FRAME_CAPTURE,0x83);
        
        # wait 3 frame periods + 10 nanoseconds for frame to be captured
        pyb.delay(1510);  # min frame speed is 2000 frames/second so 1 frame = 500 nano seconds.  so 500 x 3 + 10 = 1510

        # display the pixel data
        for i in range(ADNS3080_PIXELS_Y):
            for j in range(ADNS3080_PIXELS_X):
                regValue = self.read_register(ADNS3080_FRAME_CAPTURE)
                if( isFirstPixel and (int(regValue) & 0x40) == 0 ):
                    print("failed to find first pixel\n")

                isFirstPixel = False
                pixelValue = ( regValue << 2)
                print(pixelValue,DEC)
                if( j!= ADNS3080_PIXELS_X-1 ):
                    print(",")
                pyb.delay(50)
            print('\n')

    # Read a register from the sensor
    def read_register(self, address):
        # take the chip select low to select the device
        self._cs_pin.low()

        # send the device the register you want to read
        junk = self.spi.send_recv(address)

        # small delay
        pyb.delay(50)

        # end a value of 0 to read the first byte returned
        result = self.spi.send_recv(0x00)

        # take the chip select high to de-select
        self._cs_pin.high()

        return result

    # write a value to one of the sensor's registers
    def write_register(self, address, value):
        # take the chip select low to select the device
        self._cs_pin.low()

        # send register address
        junk = self.spi.send_recv(address | 0x80)

        # small delay
        pyb.delay(50)

        # send data
        junk = self.spi.send_recv(value);

        # take the chip select high to de-select
        self._cs_pin.high()

    # clear_motion - will cause the Delta_X, Delta_Y, and internal motion registers to be cleared
    def clear_motion(self):
        # TODO:
        pass

    # read latest values from sensor and fill in x,y and totals
    def update(self):
        # TODO:
        pass

if __name__ == "__main__":
    test = OpticalFlow()

    #test.do_test()
