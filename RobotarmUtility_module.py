"""
Robot arm Utilities Module

Functions to help control the arm

Created for Semester 1 workshops 2018
@author: Andrew Razjigaev President of QUT Robotics Club
"""
import getch, time

from math import pi

from RobotarmServo_module import pi_servo


def read_keyboard(Desired_pose,grip_value):
    '''
    Reads Keyboard presses and does logic about them
    reference:
    https://pypi.python.org/pypi/getch#downloads
    
    WARNING: only works when running function in the terminal 
    enter command in the terminal:
    python3 file_name.py
    '''
    
    (X,Y,Z,P) = Desired_pose
    #Get input key
    key = getch.getch()
    #Decide
    if key == 'k':
        on = False
        print('terminating robot arm')
    elif key == 'a':
        Y = Y + 5
    elif key == 'w':
        X = X + 5
    elif key == 's':
        X = X - 5
    elif key == 'd':
        Y = Y - 5
    elif key == 'e':
        Z = Z + 5
    elif key == 'q':
        Z = Z - 5
    elif key == 'u':
        P = P + 5*pi/180
    elif key == 'j':
        P = P - 5*pi/180
    elif key == 'g':
        #toggle open close
        if grip_value == 550:
            grip_value = 1000
        elif grip_value == 1000:
            grip_value = 550
    
    elif key == 'k':
        on = False
     
    #Update target pose after key presses
    Desired_pose = (X,Y,Z,P)

    return (Desired_pose,grip_value,on);

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
    m2.move_deg(135,bus)
    m3.move_deg(-100,bus)
    m4.move_deg(-90,bus)
    m5.move_pwm(550,bus)
    time.sleep(3)    

def Reset_robot_arm(m1,m2,m3,m4,m5,bus):
    '''
    Moves the Arm back to its reset position
    '''
    print('TERMINATING ROBOT ARM')
    Move_arm_to_home(m1,m2,m3,m4,m5,bus)
    
    m1.stop_servo(bus)
    m2.stop_servo(bus)
    m3.stop_servo(bus)
    m4.stop_servo(bus)
    m5.stop_servo(bus)

def read_arm(m1,m2,m3,m4,m5):
    '''
    Recalls the current measured angle of all servos from the last pwm feed
    reads them in radians
    '''
    q1 = m1.read_rad()
    q2 = m2.read_rad()
    q3 = m3.read_rad()
    q4 = m4.read_rad()

    return (q1,q2,q3,q4)


def Initialise_Robot_arm(bus):
    '''
    Initialises and calibrates each servo in the arm as well as defines joint limits
    '''
    #initialise each joint
    m1 = pi_servo()
    m1._init_channel(0)
    m1._calibrate_servo(-45,0,45)
    m1.define_jointlimits(-45,45)

    m2 = pi_servo()
    m2._init_channel(1)
    m2._calibrate_servo(45,90,135)  
    m2.define_jointlimits(45,135)

    m3 = pi_servo()
    m3._init_channel(2)
    m3._calibrate_servo(-25,-60,-105)
    m3.define_jointlimits(-25,-105)

    m4 = pi_servo()
    m4._init_channel(3)
    m4._calibrate_servo(-100,-55,-10)
    m4.define_jointlimits(-100,-10)

    gripper = pi_servo()
    gripper._init_channel(4)

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