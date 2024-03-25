#################################################################
# line_sensor.py
# 
# task: create a line sensor class by filling in the functions
#       __init__() and LineSensor.read(). LineSensor.read() should
#       return one byte of data that contains the readings from the 7 reflectance
#       sensors and the control bit value. While D0 in the byte is always the control
#       bit value, we do not use this value when interpreting the data; we only use it
#       to turn the reflectance sensor's LEDs on/off.
#
# init args: sda_pin - pin# connected to the MBot control board's I2C SDA pin
#            scl_pin - pin# connected to the MBot control board's I2C SCL pin
#
##################################################################

from machine import I2C, Pin
import time
import utime

INPUT_PORT = 0x00
OUTPUT_PORT = 0x01 
POLARITY_PORT = 0x02 
CONF_PORT = 0x03
DEV_ADDR = 0x18

class LineSensor:
    def __init__(self, sda_pin, scl_pin):
        '''
        In the __init__() method, instantiate an I2C object and then write to the configuration, 
        polarity, and output registers to set up your line sensor board to read from its 
        reflectance sensors. Set the configuration register such that the 7 reflectance 
        sensor bits are inputs, and set the bit that turns the line sensors on/off (LSB IO0) 
        to be an output. Set the polarity register such that the input from the 7 reflectance 
        sensors are inverted, and such that the reflectance sensor control bit retains its original 
        polarity. Hint: using bytearray([])when composing the buffer (buffer includes the register address and data to be sent).
        '''
        # Instantiate an I2c object with sda_pin and scl_pin; id = 0; don't specify the frequency or timeout values for the I2C peripheral
        self.i2c = I2C(0, sda=Pin(sda_pin), scl=Pin(scl_pin))
        
        # D1-D7 are inputs, D0 is output
        # 1111 1110 in binary; D0 as output, D1-D7 as inputs
        config_buffer = 0xFE
        self.i2c.writeto(DEV_ADDR, bytearray([CONF_PORT, config_buffer]))
        
        # set polarity of inputs; D1-D7 flip polarity, D0 retain original polarity
        # 1111 1110 in binary; D0 as original, D1-D7 as inverted
        polarity_buffer = 0xFE
        self.i2c.writeto(DEV_ADDR, bytearray([POLARITY_PORT, polarity_buffer]))
        
        # Set D0 off (LEDs off)
        self.i2c.writeto(DEV_ADDR, bytearray([OUTPUT_PORT, 0x00]))
        
        
    def read(self):
        # Turn on the LEDs in the line sensor
        self.i2c.writeto(DEV_ADDR, bytearray([OUTPUT_PORT, 0x01]))
        # allow time for sensors to turn on
        time.sleep_ms(20)
        # Read one byte of data from the I/O expander, this data is D0 = LED control pin; D1-D7 = reflectance sensor signals
        # Specify the register from which to read
        self.i2c.writeto(DEV_ADDR, bytearray([INPUT_PORT]))
        # Read the data
        data = self.i2c.readfrom(DEV_ADDR, 1)
        byte = data[0]
        # turn off the LEDs in the line sensor
        self.i2c.writeto(DEV_ADDR, bytearray([OUTPUT_PORT, 0x00]))
        return byte
        

if __name__ == "__main__":
        # write some test code to print the byte read from the I/O expander (do this in a while loop with a sleep statement for regulating code timing).
        # Hold your line sensor over a section of test track and check to make sure your reflectance sensors are reading high when they are over a white line,
        # and reading low when they are over the black line.
        SDA_PIN = 4
        SCL_PIN = 5
        line_sensor = LineSensor(SDA_PIN, SCL_PIN)
        while True:
            utime.sleep_ms(20)
            data = (line_sensor.read() >> 1)
            print(bin(data))
            
            
            
        