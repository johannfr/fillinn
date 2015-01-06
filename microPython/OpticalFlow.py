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

CLOCK_SPEED_USED = 4000000                      # This speed seams to work fine

ADNS3080_FRAME_CAPTURE = 0x13

ADNS3080_PRODUCT_ID = 0x00

ADNS3080_RESET =       'Y2'       #RESET

class OpticalFlow:
    def __init__(self, bus):
        if bus == 1:
            self.data_out =      'X8'        #MOSI
            self.data_in  =      'X7'        #MISO
            self.clock    =      'X6'        #SCK
            self.chip_select =   'X5'        #SS

        elif bus == 2:
            self.data_out =      'Y8'        #MOSI
            self.data_in  =      'Y7'        #MISO
            self.clock    =      'Y6'        #SCK
            self.chip_select =   'Y5'        #SS

        if( self.init(bus) == False ):
            print('Failed to initialise ADNS3080')

    def init(self, bus):
        retry = 0

        #---- Not sure if all of this is needed ----#
        #pyb.Pin(self.data_out, pyb.Pin.OUT_PP)
        #pyb.Pin(self.data_in, pyb.Pin.IN)
        #pyb.Pin(self.clock, pyb.Pin.OUT_PP)
        #-------------------------------------------#
        
        self._cs_pin = pyb.Pin(self.chip_select, pyb.Pin.OUT_PP)
        if( ADNS3080_RESET != 0):
            self._reset_pin = pyb.Pin(ADNS3080_RESET, pyb.Pin.OUT_PP)

        self._cs_pin.high()                     # disable device (Chip select is active low)

        # reset the device
        self.reset()

        # start the SPI library
        self.spi = pyb.SPI(bus, pyb.SPI.MASTER, baudrate=CLOCK_SPEED_USED, polarity=0, phase=1, firstbit=pyb.SPI.MSB)

        # check the sensor is functioning
        while( retry < 3 ):
            function = self.read_register(ADNS3080_PRODUCT_ID)
            if( function == 0x17 ):
                return True
            retry += 1
        return False

    # reset sensor by holding a pin high (or is it low?) for 10us.
    def reset(self):
        # return immediately if the reset pin is not defined
        if( ADNS3080_RESET == 0):
            return

        self._reset_pin.high()                  # reset sensor
        pyb.udelay(10)
        self._reset_pin.low()                   # return sensor to normal
        pyb.udelay(10)
        
    # get_pixel_data - captures an image from the sensor and stores it to the pixe_data array
    def get_pixel_data(self):            # TODO: send data to serPort do not print the data here
        isFirstPixel = True

        # write to frame capture register to force capture of frame
        self.write_register(ADNS3080_FRAME_CAPTURE,0x83);
        
        # wait 3 frame periods + 10 nanoseconds for frame to be captured
        pyb.udelay(1510);  # min frame speed is 2000 frames/second so 1 frame = 500 nano seconds.  so 500 x 3 + 10 = 1510

        data = "[["
        # display the pixel data
        for i in range(ADNS3080_PIXELS_Y):
            for j in range(ADNS3080_PIXELS_X):
                regValue = self.read_register(ADNS3080_FRAME_CAPTURE)

                if( isFirstPixel and (regValue & 0x40) == 0 ):
                    print("failed to find first pixel\n")

                isFirstPixel = False
                pixelValue = ( regValue << 2)
                data += str(pixelValue)                       # Used to be -> pixelValue,DEC not sure what the ,DEC did do
                if( j!= ADNS3080_PIXELS_X-1 ):
                    data += ","
                pyb.udelay(50)

            data += "\n"
        data += "]]"
        return data

    # Read a register from the sensor
    def read_register(self, address):
        # take the chip select low to select the device
        self._cs_pin.low()

        # send the device the register you want to read
        junk = self.spi.send_recv((address).to_bytes(1))

        # small delay
        pyb.udelay(50)

        # end a value of 0 to read the first byte returned
        result = self.spi.send_recv((0x00).to_bytes(1))

        # take the chip select high to de-select
        self._cs_pin.high()

        return int.from_bytes(result)

    # write a value to one of the sensor's registers
    def write_register(self, address, value):                       # TODO: Check if (int).to_bytes(1) is needed
        # take the chip select low to select the device
        self._cs_pin.low()

        # send register address
        junk = self.spi.send_recv((address | 0x80).to_bytes(1))

        # small delay
        pyb.udelay(50)

        # send data
        junk = self.spi.send_recv((value).to_bytes(1));

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
    optFlow = OpticalFlow(2)           # Bus 2 for the Y position
    