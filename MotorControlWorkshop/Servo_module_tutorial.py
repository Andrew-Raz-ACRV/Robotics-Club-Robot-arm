"""
Robot arm Servo class Module
Based on the Sparkfun Raspberry Pi Servo Hat in Example.py
Created for Semester 1 workshops 2018
@author: Andrew Razjigaev President of QUT Robotics Club

REFERENCE:
https://learn.sparkfun.com/tutorials/pi-servo-hat-hookup-guide

"""

import smbus, time, math
bus = smbus.SMBus(1) # the chip is on bus 1 of the available I2C buses

addr = 0x40  # I2C address of the PWM chip.

'''
Next, we want to enable the PWM chip and tell it to automatically increment 
addresses after a write (that lets us do single-operation multi-byte writes).
'''

def initialise_piHat():
    #initialise The I2C pi hat configurations
    #WRITE CODE HERE     # enable the chip
    #WRITE CODE HERE  # configure the chip for multi-byte write

class pi_servo:
    """
    This is a Class for the servo motors on the Sparkfun Raspberry PI servo hat
    it has a simpler function to initialise the servo
    it has a calibration function that helps map PWM signals to degrees
    it has a function to move the arm to a degree or a radian
    it has a function to track the last PWM thus angle input to the arm
    """
    
    dt = 0.01 # seconds for motor update
    
    #FUNCTION ASSIGNS START AND STOP ADDRESSES AUTOMATICALLY
    #By incrementing the value by 4
    
    def _init_channel(self,channel_number):
            self.channel = #WRITE CODE HERE
            self.start = #WRITE CODE HERE
            self.stop = #WRITE CODE HERE

    #functions to turn on and turn off the motor
    
    def fstart(self,bus):
        #WRITE CODE HERE

    def fstop(self,bus):
        #WRITE CODE HERE

    #Calibration Variables for the Joint Limit

    def _calibrate_servo(self,min_deg,mid_deg,max_deg):
        '''
        Test the servos before hand with values 836 and 1664
        measure their corresponding values 
        check PWM 1250 and see if you get and angle in between them.
        '''
        #Define two limits for the working region 
        self.max_limit = max_deg
        self.neutral = mid_deg
        self.min_limit = min_deg

        #They relate to the PWMs 1664, 1250 and 836
        self.max_PWM = 1664 # 2.0ms
        self.mid_PWM = 1250 # Neutral 1.5ms
        self.min_PWM = 836 # 1 ms  

    #JOINT LIMIT DEFINITION FUNCTION

    def define_jointlimits(self,minimum,maximum):
        '''
        You can define you're own minimum and maximum joint limits here based 
        on measuring the output angle in degrees with your PWM signals
        '''
        self.absolute_min = minimum
        self.absolute_max = maximum

        
    '''

    MAPPING DEGREES TO PWM

    '''

    def deg2PWM(self,deg):
        #Linear relationship constants from calibration access them like this: self.max_limit
        
        m = #WRITE CODE HERE for the linear relationship based on the calibrate servo function
        c = #WRITE CODE HERE



        #JOINT LIMIT LOGIC

        if m < 0:  #m is negative gradient
            #the difference is to reverse our interpretation
            #of max limit and min limit because of the gradient
            
            if deg>self.absolute_min:
                #upper Limit
                angle = self.absolute_min 
                
            elif deg<self.absolute_max:
                #lower limit
                angle = self.absolute_max
                
            else:
                angle = deg

        else: #m is positive gradient default

        #WRITE CODE HERE an if statement like above on joint limit logic
                

        #Linear relationship mapping angle to PWM    
        PWM = m*angle + c        

        #Record the inverse mapping to read the actual angle input
        self.last_deg = #WRITE CODE HERE

        #give the pwm signal corresponding to the angle
        return PWM

    '''

    OUTPUT FUNCTIONS FOR MOVING THE SERVOS

    '''
    def move_pwm(self,pwm,bus):
        #always round just in case, get integer
        PWM = int(round(pwm))
        
        #Gives a pwm signal to the bus
          #WRITE CODE HERE
        
        #RECORD THE PWM OUTPUT
        self.last_pwm = PWM

        #Do inverse mapping to find the angle that PWM gave
        m = #WRITE CODE HERE
        c = #WRITE CODE HERE
        

        #RECORD THE ANGLE OUTPUT
        self.last_deg = (PWM - c)/m

    def move_deg(self,degrees,bus):
        #Gives a pwm signal to the bus based on a degree input
        PWM_val = self.deg2PWM(degrees)

        #Output that PWM
        self.move_pwm(round(PWM_val),bus)

    def move_rad(self,radians,bus):
        #Gives a pwm signal to the bus based on a radians input
        #Convert radians to degrees

        degrees = radians*(180/math.pi)

        #compute PWM
        PWM_val = self.deg2PWM(degrees)

        #Output that PWM
        self.move_pwm(round(PWM_val),bus)

        

    '''

    READING OUTPUT FUNCTIONS FOR TRACKING JOINT ANGLE

    '''

    def read_pwm(self):
        #Returns the last given pwm
        return self.last_pwm

    def read_deg(self):
        #Returns the last given degree
        return self.last_deg

    def read_rad(self):
        #Returns the last given radian
        radian = self.last_deg*(math.pi/180) 

        return radian        
