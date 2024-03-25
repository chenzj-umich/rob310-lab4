import utime
from bangbang import BangBang
from pid_controller import PID
from lineSensor import LineSensor
from encoder import Encoder
from motor_class import Motor
#from speed_smoothing_filter import SmoothingFilter
#from mbot_defs import *
from wheel_speed import WheelSpeedCalculator as wsc

# Wheel speed PID parameters
K_P = 0.7       ## TUNE THESE VALUES !!!!
K_I = 0.01
K_D = 0.003

LEFT_MOTOR_POLARITY = -1 
RIGHT_MOTOR_POLARITY = 1

LEFT_ENC_POLARITY = -1
RIGHT_ENC_POLARITY = 1

ALPHA = 0.7 # smoothing parameter

CONV = 1/(20.0 * 78.0)   # GEAR RATIO / ENCODER CPR CONVERZAION FACTOR; converts from encoder counts to motor output revs

if __name__ == "__main__":

    # initialize motors, encoders, controllers, add other initialization variables and class objects
    motorL = Motor(2, 14)
    motorR = Motor(3, 15)
    encL = Encoder(6, 7)
    encR = Encoder(8, 9)
    line_sensor = LineSensor(4, 5)
    bang = BangBang(line_sensor)
    encL_read = 0
    encR_read = 0
    pidL = PID(P=K_P, I=K_I, D=K_D, setpoint=0.25)
    pidR = PID(P=K_P, I=K_I, D=K_D, setpoint=0.25)
    
    try:
        
        while True:
            ## sleep the loop to allow encoders to create tick delta
            start_time = utime.ticks_ms()
            calcL = wsc(encL_read, start_time)
            calcR = wsc(encR_read, start_time)
            utime.sleep_ms(50)
            ##
            
            ## calculate time delta between loop start and present time
            current_time = utime.ticks_ms()
            dt = utime.ticks_diff(current_time, start_time)/1000
            ##
            
            ## calculate left and right wheel speeds using encoder tick delta and time delta
            ## you might need some code to handle the loop initialization when dt = 0
            encL_read = encL.read() * LEFT_ENC_POLARITY
            speedL = calcL.calculateSpeed(encL_read, current_time)
            encR_read = encR.read() * RIGHT_ENC_POLARITY
            speedR = calcR.calculateSpeed(encR_read, current_time)
            ##
            
            ## update the bang-bang controller and set the left and right wheel speed setpoints
            [L_setpoint, R_setpoint] = bang.update()
            ##
            
            ## OPTIONAL: apply the smoothing/ low-pass filter to the wheel speed setpoint
            
            ##
            
            ## set the PID controller wheel speed setpoints
            pidL.set_speed(L_setpoint)
            pidR.set_speed(R_setpoint)
            ##
            
            ## calculate the wheel speed errors
            errorL = L_setpoint - speedL
            errorR = R_setpoint - speedR
            ##
            
            ## update the PID controller and return the left and right motor duty cycles
            pid_inputL = pidL.update(errorL, dt)
            pid_inputR = pidR.update(errorR, dt)
            ##
            
            ## IMPORTANT: ADD SATURATION LIMIT TO MOTOR DUTY CYCLES
            
            if pid_inputL > 1:
                pid_inputL = 0.99
            elif pid_inputL < -1:
                pid_inpitL = -0.99
            
            if pid_inputR > 1:
                pid_inputR = 0.99
            elif pid_inputR < -1:
                pid_inpitR = -0.99
            ##
            
            ## set saturation limited PID duty cycle output to motor PWM duty cycle
            motorL.set(pid_inputL * LEFT_MOTOR_POLARITY)
            motorR.set(pid_inputR * RIGHT_MOTOR_POLARITY)
            ##
            
    except KeyboardInterrupt:
        left_motor.set(0) # turn off motors
        right_motor.set(0)
        print("Loop interrupted by Ctrl+C")         
            
        