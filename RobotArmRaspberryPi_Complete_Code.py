#!usr/bin/env python
"""
Robot arm Control Script
Script to control the Workshop arms so that they go to a point
Created for Semester 1 workshops 2018
@author: Andrew Razjigaev President of QUT Robotics Club
"""

import smbus, numpy, time, getch

from math import sin, cos, pi

from RobotarmServo_module import pi_servo, initialise_piHat

from RobotarmKinematics_module import update_joints, DH_matrix, ForwardKinematics, compute_error, compute_Jacobian, damped_least_squares

from RobotarmUtility_module import read_keyboard, move_arm, Move_arm_to_home, Reset_robot_arm, read_arm, Initialise_Robot_arm, robotarmLimits


# ***** MAIN ***** #
if __name__ == "__main__":

    print('STARTING ROBOT ARM...')
    
    ####INITIALISATION####
    #Initialise the Pi hat
    bus = smbus.SMBus(1) # the chip is on bus 1 of the available I2C buses
    addr = 0x40  # I2C address of the PWM chip.
    
    #Initialises the whole robot arm
    (m1,m2,m3,m4,m5) = Initialise_Robot_arm(bus)
    
    #Start arm at home position
    Move_arm_to_home(m1,m2,m3,m4,m5,bus)

    ####DEFINITIONS####
    #Measure arm segments l1,l2,l3,l4
    arm_lengths = (100,120,140,140)

    #Determine initial position set the desired pose there
    joint_values = read_arm(m1,m2,m3,m4,m5)
    
    Desired_pose = ForwardKinematics(joint_values,arm_lengths)

    #While loop for the control
    on = True
    #set gripper to initial closed pwm
    grip_value = 550

    print('Running loop ... Press keys now')
    
    ####CONTROL LOOP####
    #Run loop to follow the desired pose updated by the keyboard
    while(on):
        
        #Read the keyboard presses and compute the new pose and gripper and
        #whether the user killed the robot
        (Desired_pose,grip_value,on) = read_keyboard(Desired_pose,grip_value,on)
        
        #Read the current joint angles
        joint_values = read_arm(m1,m2,m3,m4,m5)
        
        #Compute the Current tool point
        tool_point = ForwardKinematics(joint_values,arm_lengths)
        
        #compute the error vector between desired and current toolpoint
        dX = compute_error(Desired_pose,tool_point)
        
        #compute the Jacobian / derivative of tool and joint
        J = compute_Jacobian(joint_values,arm_lengths)
        
        #Retrieve joint limits:
        (joint_mins, joint_maxs) = robotarmLimits(m1,m2,m3,m4)
        
        #Compute the inverse Jacobian 
        inv_J = damped_least_squares(J, joint_values, joint_mins, joint_maxs)
        
        #Estimate the required joint update to reduce positon error
        dQ = numpy.dot(inv_J,dX)
        print(dQ)
        #update the joints
        new_joint_values = update_joints(joint_values,dQ)
        
        #Move the robot arm
        move_arm(new_joint_values,grip_value,m1,m2,m3,m4,m5,bus)

        #Reset the desired pose at the tooltip
        (Desired_pose) = ForwardKinematics(joint_values,arm_lengths)
        
    #When the user kills the while loop, reset the robot and turn off 
    Reset_robot_arm(m1,m2,m3,m4,m5,bus)
    print('ROBOT ARM PROGRAM HAS ENDED')
