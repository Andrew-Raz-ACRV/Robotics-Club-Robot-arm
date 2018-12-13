"""
Robot arm Utilities Module
Functions to help control the arm
Created for Semester 2 workshops 2018
@author: Andrew Razjigaev President of QUT Robotics Club
"""
import time

from math import pi, degrees

from RobotarmServo_module import pi_servo
    
def move_arm(new_joint_values,grip_value,m1,m2,m3,m4,m5,bus):
    '''
    Moves all the servos based on the computed joint values in radians and moves
    the gripper based on the grip value
    '''
    (q1,q2,q3,q4) = new_joint_values
    
    m1.move_rad(q1,bus)
    m2.move_rad(q2,bus)
    m3.move_rad(q3,bus)
    m4.move_rad(q4,bus)
    m5.move_pwm(grip_value,bus)

def Move_arm_to_home(m1,m2,m3,m4,m5,bus):
    '''
    Move all servos to home default angle
    '''
    print('Moving to home position')
    m1.move_deg(0,bus)
    time.sleep(0.5) 
    m2.move_deg(135,bus)
    time.sleep(0.5) 
    m3.move_deg(-100,bus)
    time.sleep(0.5)
    m4.move_deg(-90,bus)
    time.sleep(0.5)
    m5.move_pwm(410,bus)
    time.sleep(0.5)   

def Reset_robot_arm(m1,m2,m3,m4,m5,bus):
    '''
    Moves the Arm back to its reset position
    '''
    print('TERMINATING ROBOT ARM')
    Move_arm_to_home(m1,m2,m3,m4,m5,bus)
    
    m1.fstop(bus)
    m2.fstop(bus)
    m3.fstop(bus)
    m4.fstop(bus)
    m5.fstop(bus)

def read_arm(m1,m2,m3,m4):
    '''
    Recalls the current measured angle of all servos from the last pwm feed
    reads them in radians
    '''
    q1 = m1.read_rad()
    q2 = m2.read_rad()
    q3 = m3.read_rad()
    q4 = m4.read_rad()
    
    #print(degrees(q1),degrees(q2),degrees(q3),degrees(q4))
    
    return (q1,q2,q3,q4)


def Initialise_Robot_arm(bus):
    '''
    Initialises and calibrates each servo in the arm as well as defines joint limits
    '''
    #initialise each joint
    m1 = pi_servo()
    m1._init_channel(0)
    m1._calibrate_servo(-45,0,45)
    m1.define_jointlimits(-60,60)

    m2 = pi_servo()
    m2._init_channel(1)
    m2._calibrate_servo(55,96,141)  
    m2.define_jointlimits(20,135)

    m3 = pi_servo()
    m3._init_channel(2)
    m3._calibrate_servo(-25,-65,-110) 
    m3.define_jointlimits(-120,-20)

    m4 = pi_servo()
    m4._init_channel(3)
    m4._calibrate_servo(-90,-45,-5) 
    m4.define_jointlimits(-100,0)

    gripper = pi_servo()
    gripper._init_channel(4)
    gripper._calibrate_servo(0,30,60)
    gripper.define_jointlimits(0,60)

    #Start each joint
    m1.fstart(bus)
    m2.fstart(bus)
    m3.fstart(bus)
    m4.fstart(bus)
    gripper.fstart(bus)
    
    return (m1,m2,m3,m4,gripper);

def robotarmLimits(m1,m2,m3,m4):
    '''
    gets the joint limits from the arm as tuples
    '''    
    joint_mins = (m1.absolute_min*(pi/180), m2.absolute_min*(pi/180),
                  m3.absolute_min*(pi/180), m4.absolute_min*(pi/180))
    
    joint_maxs = (m1.absolute_max*(pi/180), m2.absolute_max*(pi/180),
                  m3.absolute_max*(pi/180), m4.absolute_max*(pi/180))

    return (joint_mins, joint_maxs);

