"""
Robot arm Servo class Module
Based on the Sparkfun Raspberry Pi Servo Hat in Example.py
Created for Semester 1 workshops 2018
@author: Andrew Razjigaev President of QUT Robotics Club
About PWM from the Sparkfun Hat:
    By default, the PWM frequency of the chip is 200Hz, or one pulse every 5ms. 
    The start time register determines when the pulse goes high in the 5ms 
    cycle. All channels are synchronized to that cycle. Generally, this should 
    be written to 0. The second write is to the “stop time” register, and it 
    controls when the pulse should go low. The range for this value is from 0 
    to 4095, and each count represents one slice of that 5ms period (5ms/4095), 
    or about 1.2us. Thus, the value of 1250 written above represents about 
    1.5ms of high time per 5ms period.
    
    Servo motors get their control signal from that pulse width. Generally 
    speaking, a pulse width of 1.5ms yields a “neutral” position, halfway
    between the extremes of the motor’s range. 1.0ms yields approximately 
    90 degrees off center, and 2.0ms yields -90 degrees off center. 
    In practice, those values may be slightly more or less than 90 degrees, 
    and the motor may be capable of slightly more or less than 90 degrees of 
    motion in either direction.
    
    To address other channels, simply increase the address of the two registers
    above by 4. Thus, start time for channel 1 is 0x0A, for channel 2 is 0x0E, 
    channel 3 is 0x12, etc. and stop time address for channel 1 is 0x0C, for 
    channel 2 is 0x10, channel 3 is 0x14, etc.
    
    The 1.5ms end time is equal to 1.2us per count. This represents the neutral
    position of the servo, midway between both extremes. Each degree of
    deviation from neutral requires that number (1250) to be changed by 4.6.
    Thus, the 90 degree offset from neutral requires that 414 counts (90*4.6)
    be added or subtracted from 1250.  
    These are 836 and 1664 which is why we consider these values as the 
    default working limits; however, one can test to see how far their servo 
    goes.
    
    If you write a 0 to the start address, every degree of offset from 
    90 degrees requires 4.6 counts written to the stop address. 
    In other words, multiply the number of degrees offset from neutral you 
    wish to achieve by 4.6, then either add or subtract that result from 1250, 
    depending on the direction of motion you wish. For example, a 45 degree 
    offset from center would be 207 (45x4.6) counts either more or less than 
    1250, depending upon the direction you desire the motion to be in.
    
    REFERENCE:
    https://learn.sparkfun.com/tutorials/pi-servo-hat-hookup-guide
    
    
    
First point: in most OS level interactions, the I2C communication bus is 
referred to as SMBus. Thus we get our first lines of code. 
This imports the smbus module, creates an object of type SMBus, and attaches it 
to bus “1” of the Pi’s various SMBuses. We also need time and math for later 
use
We have to tell the program the part’s address. By default, it is 0x40, so set 
a variable to that for later use as well.
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
    bus = smbus.SMBus(1)  # the chip is on bus 1 of the available I2C buses
    addr = 0x20           # I2C address of the PWM chip.
    bus.write_byte_data(addr, 0, 0x20)     # enable the chip
    bus.write_byte_data(addr, 0xfe, 0x1e)  # configure the chip for multi-byte write
    
    return (bus,addr);
      

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
            if deg<self.absolute_min:
                #lower Limit
                angle = self.absolute_min
            elif deg>self.absolute_max:
                #Upper limit
                angle = self.absolute_max
            else:
                angle = deg
                
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
        #always round just in case, get integer
        PWM = int(round(pwm))
        
        #Gives a pwm signal to the bus
        bus.write_word_data(addr, self.stop, PWM) 
        
        #RECORD THE PWM OUTPUT
        self.last_pwm = PWM
        
        #Do inverse mapping to find the angle that PWM gave
        m = (self.max_PWM - self.min_PWM)/(self.max_limit - self.min_limit)
        c = self.max_PWM - m*(self.max_limit)
        
        #RECORD THE ANGLE OUTPUT
        self.last_deg = (PWM - c)/m

        #Give the controller some time to move
        #time.sleep(self.dt)   
        
        

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
