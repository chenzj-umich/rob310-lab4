##############################################################
#  pid.py
#
#  task: create a PID controller class by filling in the functions
#  
#  init args: P - proportional gain value
#             I - intergral gain value
#             D - derivative gain value
#             setpoint - wheel speed setpoint for the controller
# 
#
#############################################################

import utime
from encoder_class import Encoder
from motor_class import Motor
#from speed_smoothing_filter import SmoothingFilter
#from mbot_defs import *
import asyncio
import machine
from wheel_speed import WheelSpeedCalculator as wsc

# Wheel speed PID test gains; keep the gains within the following respective orders of magnitude:
K_P = 0.3
K_I = 0.01
K_D = 0.0005

LEFT_MOTOR_POLARITY = 1 # apply these to your wheel duty cycles to control the direction that they spin
RIGHT_MOTOR_POLARITY = -1

ALPHA = 0.75 # smoothing parameter
CONV = 1/(20.0 * 78.0)   # GEAR RATIO / ENCODER CPR CONVERZAION FACTOR; converts from encoder counts to motor output revs

class PID:
    def __init__(self, P=0.0, I=0.0, D=0.0, setpoint=0.0):
        # create attributes for K_p,
                              #  K_i,
                              #  K_d,
                              #  speed setpoint,
                              #  previous error (initialize to 0),
                              #  the controllers integral term; initialize to zero (this builds up with each controller update)
        self.kp = P
        self.ki = I
        self.kd = D
        self.setpoint = setpoint
        self.prev_err = 0.0
        self.integral = 0.0
       
    
    def update(self, error, dt):

        # calculate the controller's proportional term
        P = self.kp * error
        
        # calculate the controller's integral term
        self.integral += self.ki * dt * error
        
        # calculate the controller's derivative term
        D = self.kd * (error - self.prev_err) / dt
        
        # store error in the previous error attribute for calculating the derivative term in the next controller update
        self.prev_err = error
        
        return (P + self.integral + D) # PID output is a duty cycle
    
    def set_speed(self, setpoint):
        # update the self.setpoint attribute to the setpoint argument
        self.setpoint = setpoint
        # set the previous error attribute to 0. Since the setpoint has changed the present self.prev_error value is no longer accurate
        self.prev_error = 0
        
    def setP(self, P):
        self.Kp = P
        
    def setI(self, I):
        self.Ki = I
        
    def setD(self, D):
        self.Ki = D

if __name__ == "__main__":
    
    # write some code to test your PID controller
    setpoint = 0.5
    pid = PID(P=K_P, I=K_I, D=K_D, setpoint=setpoint)
    enc = Encoder(6, 7)
    motor = Motor(2, 14)
    enc_count = enc.read()
    
    try:
        start_time = utime.ticks_ms()
        calc = wsc(0, start_time)
        while True:
            utime.sleep_ms(50)
            current_time = utime.ticks_ms()
            dt = utime.ticks_diff(current_time, start_time)/1000
            
            speed = calc.calculateSpeed(enc.read(), current_time)
            
            pid_input = pid.update(setpoint - speed, dt)
            if pid_input > 1:
                pid_input = 0.99
            elif pid_input < -1:
                pid_input = -0.99
            print(f'error: {setpoint - speed}; input: {pid_input}; enc: {enc.read()}')
            motor.set(pid_input * -1)
            m0_csns = machine.ADC(26)
            data = m0_csns.read_u16()
            current = data/65535 * 3.1 
            #print(f'motor_current: {current}')
    except KeyboardInterrupt:
        motor.set(0)
        print('motors turend off')
    