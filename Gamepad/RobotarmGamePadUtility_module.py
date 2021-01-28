"""
Robot arm Utilities Module
Functions to help control the arm
Created for Semester 1 workshops 2018
@author: Andrew Razjigaev President of QUT Robotics Club
"""
import time

from math import pi, degrees

from RobotarmServo_module import pi_servo

from evdev import InputDevice, categorize, ecodes

def read_gamepad(gamepad,Desired_pose,grip_value,on,closed_value,open_value,reset,hold):
    '''
    Reads gamepad presses and converts that into toolpoint position updates
    '''
    #Extract components of Pose
    (X,Y,Z,P) = Desired_pose

    #Extract components of hold
    (L,R,U,D) = hold

    #button code Mapping
    A_press = 289
    B_press = 290
    X_press = 288
    Y_press = 291

    Start_press = 297
    Select_press = 296

    Right_trig = 293
    Left_trig = 292
    
    #Logic from event to Button Type to robot arm action:

    #Read gamepad latest signal
    event = gamepad.read_one()
    if event is not None:       

        #Decide what signal it is:
        if event.type == ecodes.EV_KEY:
            #Button
            if event.value==1:
                if event.code==A_press:
                    #print("A")
                    #Open gripper
                    grip_value = open_value           
                    
                elif event.code==B_press:
                    #print("B")
                    #Close gripper
                    grip_value = closed_value
                    
                elif event.code==X_press:
                    #print("X")
                    P = P + 5*pi/180
                    
                elif event.code==Y_press:
                    #print("Y")
                    P = P - 5*pi/180
                    
                elif event.code==Start_press:
                    #print("Start")
                    on = False
                    
                elif event.code==Select_press:
                    #print("Select")
                    reset = True
                    
                elif event.code==Right_trig:
                    #print("Right Trigger")
                    Z = Z - 5

                    
                elif event.code==Left_trig:
                    #print("Left Trigger")
                    Z = Z + 5
        
        elif event.code==1 and event.type==3:
            #Up or down buttons
            if event.value==255:
                #print("down")
                D = 1
            elif event.value==0:
                #print("up")
                U = 1
            elif event.value==127:
                #hold off
                U = 0
                D = 0

                
        elif event.code==0 and event.type==3:
            #left or right buttons
            if event.value==0:
                #print("left")
                L = 1
            elif event.value==255:
                #print("right")
                R = 1
            elif event.value==127:
                #hold off
                L = 0
                R = 0

    #Take hold effects:
    if U==1:
        X = X + 1
    elif D==1:
        X = X - 1
    elif L==1:
        Y = Y + 1
    elif R==1:
        Y = Y - 1

                
    #Update target pose after key presses
    Desired_pose = (X,Y,Z,P)
    #wrap up the hold buttons
    hold = (L,R,U,D)

    return (Desired_pose,grip_value,on,reset,hold);

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

def Move_arm_to_home(m1,m2,m3,m4,m5,closed_grip_value,bus):
    '''
    Move all servos to home default angle
    '''
    print('Moving to home position')
    m1.move_deg(0,bus)
    time.sleep(0.5) 
    m2.move_deg(135,bus)
    time.sleep(0.5) 
    m3.move_deg(-90,bus)
    time.sleep(0.5)
    m4.move_deg(-100,bus)
    time.sleep(0.5)
    m5.move_pwm(closed_grip_value,bus)
    time.sleep(0.5)   

def Reset_robot_arm(m1,m2,m3,m4,m5,closed_grip_value,bus):
    '''
    Moves the Arm back to its reset position
    '''
    Move_arm_to_home(m1,m2,m3,m4,m5,closed_grip_value,bus)
    
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
    m3._calibrate_servo(-15,-47,-95) #-30, -75, -125
    m3.define_jointlimits(-95,-15)

    m4 = pi_servo()
    m4._init_channel(3)
    m4._calibrate_servo(-90,-50,-5) 
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

