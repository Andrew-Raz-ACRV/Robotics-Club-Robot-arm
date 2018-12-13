#!usr/bin/env python
"""
Robot arm Servo class Module
Based on the Sparkfun Raspberry Pi Servo Hat in Example.py
Created for Semester 1 workshops 2018
@author: Andrew Razjigaev President of QUT Robotics Club
"""
import smbus, time, math
bus = smbus.SMBus(1) # the chip is on bus 1 of the available I2C buses

addr = 0x40  # I2C address of the PWM chip.
'''
Next, we want to enable the PWM chip and tell it to automatically increment 
addresses after a write (that lets us do single-operation multi-byte writes).
'''
def initialise_piHat():
    #initialise The I2C pi hat
    bus.write_byte_data(addr, 0, 0x20)     # enable the chip
    bus.write_byte_data(addr, 0xfe, 0x1e)  # configure the chip for multi-byte write  
      

class pi_servo:
    """
    This is a Class for the servo motors on the Sparkfun Raspberry PI servo hat
    it has a simpler function to initialise the servo
    it has a calibration function that helps map PWM signals to degrees
    it has a function to move the arm to a degree or a radian
    it has a function to track the last PWM thus angle input to the arm
    """
    
    #dt = 0.01 # seconds for motor update
    
    #FUNCTION ASSIGNS START AND STOP ADDRESSES AUTOMATICALLY
    #By incrementing the value by 4
    def _init_channel(self,channel_number):
            self.channel = 0 + channel_number
            self.start = 0x06 + 4*channel_number
            self.stop = 0x08 + 4*channel_number

    #functions to turn on and turn off the motor
    def fstart(self,bus):
        bus.write_word_data(addr, self.start, 0) 

    def fstop(self,bus):
        bus.write_word_data(addr, self.stop, 0)
           
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
       
        #Linear relationship constants
        m = (self.max_PWM - self.min_PWM)/(self.max_limit - self.min_limit)
        c = self.max_PWM - m*(self.max_limit)

        #JOINT LIMIT LOGIC
        if deg<self.absolute_min:
            #lower Limit
            angle = self.absolute_min
            #print('m>0 lower limit',angle,' degree',deg)
            #print('limits are:',self.absolute_min,self.absolute_max)
        elif deg>self.absolute_max:
            #Upper limit
            angle = self.absolute_max
            #print('m>0 upper limit',angle,' degree',deg)
            #print('limits are:',self.absolute_min,self.absolute_max)
        else:
            angle = deg
            #print('m>0 in range',angle,' degree',deg)
            #print('limits are:',self.absolute_min,self.absolute_max)
                    
                
        #Linear relationship mapping angle to PWM    
        PWM = m*angle + c        

        #Record the inverse mapping to read the actual angle input
        self.last_deg = (PWM - c)/m

        #give the pwm signal corresponding to the angle
        return PWM

    '''
    OUTPUT FUNCTIONS FOR MOVING THE SERVOS
    '''
    def move_pwm(self,pwm,bus):

        #RECORD THE PWM OUTPUT
        self.last_pwm = pwm
        
        #Do inverse mapping to find the angle that PWM gave
        m = (self.max_PWM - self.min_PWM)/(self.max_limit - self.min_limit)
        c = self.max_PWM - m*(self.max_limit)
        
        #RECORD THE ANGLE OUTPUT BEFORE ROUNDING
        self.last_deg = (pwm - c)/m

        #Round PWM before output to servo, it expects integer
        PWM = int(round(pwm))
        
        #Gives a pwm signal to the bus
        bus.write_word_data(addr, self.stop, PWM) 

        #Give the controller some time to move
        #time.sleep(self.dt)   
        
        

    def move_deg(self,degrees,bus):
        #Gives a pwm signal to the bus based on a degree input
        PWM_val = self.deg2PWM(degrees)
        #Output that PWM
        self.move_pwm(PWM_val,bus)



    def move_rad(self,radians,bus):
        #Gives a pwm signal to the bus based on a radians input
        #Convert radians to degrees
        degrees = radians*(180/math.pi)
        #compute PWM
        PWM_val = self.deg2PWM(degrees)
        #Output that PWM
        self.move_pwm(PWM_val,bus)
        
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
