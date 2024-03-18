#################################################################
# motor.py
# 
# task: create a motor class by filling in the functions
#       Motor.set() should take a duty cycle from -1 to +1
#       and set the control signals to the motor driver
#
# init args: pwm_pin - pin# connected to PWM input of motor driver
#            dir_pin - pin# connected to DIR input of motor driver
#
##################################################################

from machine import Pin, PWM
from encoder_class import Encoder
import utime

class Motor:
    def __init__(self, pwm_pin, dir_pin):
        ## add the pins for motor direction and pwm
        ## also set the PWM frequency and set the initial duty cycle to zero
        self.pwm = PWM(pwm_pin, freq=10000, duty_u16=0)
        self.dir_pin = Pin(dir_pin, Pin.OUT)
        
    def set(self, duty):
        # add the logic to set the motor direction --> hint: self.dir.on() & self.dir.off() will be useful)
        # and also set the duty cycle of the motor, duty should be normalized from -65535 to +65535
        if duty < 1 and duty >= 0:
            self.dir_pin.on()
            self.pwm.duty_u16(int(duty * 65535))
        elif duty > -1 and duty < 0:
            self.dir_pin.off()
            self.pwm.duty_u16(int(-1 * duty * 65535))
        else:
            self.pwm.duty_u16(0)
            print("ERROR: duty out of range")


    def off(self):
        # set duty to zero
        self.pwm.duty_u16(0)

if __name__ == "__main__":
    # Write some code to test your motor here
    
    try:
        enc = Encoder(6,7)
        motor = Motor(2,14)
        
        motor.set(-0.25)
        start_time = utime.ticks_ms()
        while utime.ticks_diff( utime.ticks_ms(), start_time) < 2000:
            utime.sleep_ms(20)
            print(f'Forward encoder count: {enc.read()}')
            
        speed = enc.read() * 1000 / (2000 * 20 * 78) 
        print(f"wheel speed: {speed}")
        
        """
        motor.set(0)
        start_time = utime.ticks_ms()
        while utime.ticks_diff( utime.ticks_ms(), start_time) < 2000:
            utime.sleep_ms(20)
            print(f'Stop encoder count: {enc.read()}')  
        
        motor.set(0.25)
        start_time = utime.ticks_ms()
        while utime.ticks_diff( utime.ticks_ms(), start_time) < 5000:
            utime.sleep_ms(20)
            print(f'Backward encoder count: {enc.read()}')
        """
        
    except KeyboardInterrupt:
        motor.set(0)
        print('ctrl+c pressed')