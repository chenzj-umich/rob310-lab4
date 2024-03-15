##############################################################
#  encoder.py
#
#  task: create an encoder class by filling in the functions
#  
#  args: pinA and pinB where the encoder is connected
#
#  note: to set up an IRQ that runs on both a rising
#  and falling edge on a pin do the following:
#   
#    pin.IRQ(ISR, Pin.IRQ_FALLING | Pin.IRQ_RISING)
#
#  where pin is the pin name, and ISR is the handler function 
#
#############################################################

from machine import Pin
import utime

class Encoder:
    def __init__(self, pinA, pinB):
       # initialize the class here
       self.encoderCount = 0
       self.pinA = Pin(pinA, Pin.IN)
       self.pinB = Pin(pinB, Pin.IN)
       self.pinA.irq(self.A_ISR, Pin.IRQ_FALLING | Pin.IRQ_RISING)
       self.pinB.irq(self.B_ISR, Pin.IRQ_FALLING | Pin.IRQ_RISING)

    def A_ISR(self, pin):
        # interrupt service routine for encoder channel A
        if self.pinA.value() == self.pinB.value():
            self.dec()
        else:
            self.inc()
    
    def B_ISR(self, pin):
        # service routine for encoder channel B
        if self.pinA.value() == self.pinB.value():
            self.inc()
        else:
            self.dec()
    
    def inc(self):
        # increment function
        self.encoderCount += 1
        
    def dec(self):
        # decrement function
        self.encoderCount -= 1
        
    def set(self, count):
        # set function sets counter to value of count
        self.encoderCount = count
        
    def read(self):
        # returns value in count
        return self.encoderCount

            
if __name__ == "__main__":
    # write some test code here to make sure your encoder driver works
    print("Test started!")
    encoder = Encoder(8, 9)
    while True:
        utime.sleep_ms(50)
        print(f"Encoder value: {encoder.read()}")
    