# -*- coding: utf-8 -*-
"""
Created on Sun Feb 11 21:41:36 2018

@author: Andrew
"""
import smbus, time, math

"""
Enable the PWM chip and tell it to automatically increment addresses after 
a write (that lets us do single-operation multi-byte writes).
"""
bus = smbus.SMBus(1)  # the chip is on bus 1 of the available I2C buses

addr = 0x20           # I2C address of the PWM chip.

bus.write_byte_data(addr, 0, 0x20)     # enable the chip

bus.write_byte_data(addr, 0xfe, 0x1e)  # configure the chip for multi-byte write

class Pi_servo:
    """
    Class for the servo motors on the PI hat

    """
    addr = 0x40
    
    ## The 1.5ms end time is equal to 1.2us per count. This represents the neutral
    ##  position of the servo, midway between both extremes. Each degree of
    ##  deviation from neutral requires that number (1250) to be changed by 4.6.
    ##  Thus, the 90 degree offset from neutral requires that 414 counts (90*4.6)
    ##  be added or subtracted from 1250.
    
    #Variables for the Joint Limit
    max_limit = 45
    mid_val = 0
    min_limit = -45
    
    max_PWM = 1664 # 2.0ms
    mid_PWM = 1250 # Neutral 1.5ms
    min_PWM = 836 # 1 ms
    
    dt = 0.01 # seconds for motor update
    
    #TABLE ASSIGNS START AND STOP ADDRESSES AUTOMATICALLY
    def _init_channel(self,value):
        if value == 0:
            self.channel = 0
            self.start = 0x06
            self.stop = 0x08
        elif value == 1:
            self.channel = 1
            self.start = 0x0A
            self.stop = 0x0C
        elif value == 2:
            self.channel = 2
            self.start = 0x0E
            self.stop = 0x10
        elif value == 3:
            self.channel = 3
            self.start = 0x12
            self.stop = 0x14
        elif value == 4:
            self.channel = 4
            self.start = 0x16
            self.stop = 0x18
        elif value == 5:
            self.channel = 5
            self.start = 0x1A
            self.stop = 0x1C
        elif value == 6:
            self.channel = 6
            self.start = 0x1E
            self.stop = 0x20
        elif value == 7:
            self.channel = 7
            self.start = 0x22
            self.stop = 0x24
        elif value == 8:
            self.channel = 8
            self.start = 0x26
            self.stop = 0x28
        elif value == 9:
            self.channel = 9
            self.start = 0x2A
            self.stop = 0x2C
        elif value == 10:
            self.channel = 10
            self.start = 0x2E
            self.stop = 0x30
        elif value == 11:
            self.channel = 11
            self.start = 0x32
            self.stop = 0x34
        elif value == 12:
            self.channel = 12
            self.start = 0x36
            self.stop = 0x38
        elif value == 13:
            self.channel = 13
            self.start = 0x3A
            self.stop = 0x3C
        elif value == 14:
            self.channel = 14
            self.start = 0x3E
            self.stop = 0x40
        elif value == 15:
            self.channel = 15
            self.start = 0x42
            self.stop = 0x44 
        else:
             print('ERROR pick a channel on the HAT FROM 0 TO 15')   
    
    def start_servo(self,bus):
        bus.write_word_data(self.addr, self.start, 0)
 
    def stop_servo(self,bus):
        bus.write_word_data(self.addr, self.stop, 0)

    
    def deg2PWM(self,deg):
        if deg<self.min_limit:
            #lower Limit
            PWM = self.min_PWM
        elif deg>self.max_limt:
            #Max limit
            PWM = self.max_PWM
        else:
            #Linear relationship
            m = (self.max_PWM - self.min_PWM)/(self.max_limit - self.min_limit)
            c = self.max_PWM - m*(self.max_limit)
            PWM = m*deg + c
            
        return PWM      
    
    def move_deg(self,degrees,bus):
        
        PWM_val = self.deg2PWM(degrees)
        
        bus.write_word_data(addr, self.stop, PWM_val) 
        
        time.sleep(self.dt)
        
    def move_rad(self,radians,bus):
        
        degrees = radians*(180/math.pi)
        
        PWM_val = self.deg2PWM(degrees)
        
        bus.write_word_data(addr, self.stop, PWM_val) 
        
        time.sleep(self.dt)
    
    
    
    