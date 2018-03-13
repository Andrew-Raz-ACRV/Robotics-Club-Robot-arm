import smbus, time, math

bus = smbus.SMBus(1)
addr = 0x40

class pi_servo:
    """
    Class for the servo motors on the PI hat
    """
    ## The 1.5ms end time is equal to 1.2us per count. This represents the neutral
    ##  position of the servo, midway between both extremes. Each degree of
    ##  deviation from neutral requires that number (1250) to be changed by 4.6.
    ##  Thus, the 90 degree offset from neutral requires that 414 counts (90*4.6)
    ##  be added or subtracted from 1250.
    
    #Calibration Variables for the Joint Limit
    def _calibrate_servo(self,min_deg,mid_deg,max_deg):
        self.max_limit = max_deg
        self.min_limit = min_deg
        #pwm limits
        self.max_PWM = 1664 # 2.0ms
        self.mid_PWM = 1250 # Neutral 1.5ms
        self.min_PWM = 836 # 1 ms       
    
    dt = 0.01 # seconds for motor update
    
    #TABLE ASSIGNS START AND STOP ADDRESSES AUTOMATICALLY
    def _init_channel(self,value):
            self.channel = 0 + value
            self.start = 0x06 + 4*value
            self.stop = 0x08 + 4*value

    def fstart(self,bus):
        bus.write_word_data(addr, self.start, 0)
 
    def fstop(self,bus):
        bus.write_word_data(addr, self.stop, 0)

    def deg2PWM(self,deg):
        
        #Linear relationship constants
        m = (self.max_PWM - self.min_PWM)/(self.max_limit - self.min_limit)
        c = self.max_PWM - m*(self.max_limit)

        if m < 0:  #m is negative gradient
            #difference is to reverse our interpretation
            #of max limit and min because of the gradient
            if deg>self.min_limit:
                #upper Limit
                PWM = self.min_PWM
            elif deg<self.max_limit:
                #Max limit
                PWM = self.max_PWM
            else:
                #Linear relationship
                PWM = m*deg + c

        else: #m is positive gradient expected
            if deg<self.min_limit:
                #lower Limit
                PWM = self.min_PWM
            elif deg>self.max_limit:
                #Max limit
                PWM = self.max_PWM
            else:
                #Linear relationship
                PWM = m*deg + c

        self.last_deg = (PWM - c)/m
        
        return PWM

    def move_pwm(self,pwm,bus):
        
        bus.write_word_data(addr, self.stop, pwm) 

        self.last_pwm = pwm
        
        time.sleep(self.dt)        
    
    def move_deg(self,degrees,bus):
        
        PWM_val = self.deg2PWM(degrees)
        
        bus.write_word_data(addr, self.stop, round(PWM_val))

        self.last_pwm = round(PWM_val)
        #print(round(PWM_val))
        time.sleep(self.dt)
        
    def move_rad(self,radians,bus):
        
        degrees = radians*(180/math.pi)
        
        PWM_val = self.deg2PWM(degrees)
        
        bus.write_word_data(addr, self.stop, round(PWM_val))

        self.last_pwm = round(PWM_val)
        #print(round(PWM_val))
        time.sleep(self.dt)

    def read_pwm(self):

        return self.last_pwm

    def read_deg(self):

        return self.last_deg

    def read_rad(self):

        radian = self.last_deg*(math.pi/180) 

        return radian        
