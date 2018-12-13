#!/usr/bin/python
"""
Vision Robot arm Control Script
Script to control the Workshop arms so that they go to a point
Created for Semester 2 workshops 2018
@author: Andrew Razjigaev President of QUT Robotics Club
"""

import smbus, numpy, time

from math import sin, cos, pi

from RobotarmServo_module import pi_servo, initialise_piHat

from RobotarmKinematics_module import update_joints, DH_matrix, ForwardKinematics, measure_position_error, compute_error, compute_Jacobian, damped_least_squares

from RobotarmUtility_module import move_arm, Move_arm_to_home, Reset_robot_arm, read_arm, Initialise_Robot_arm, robotarmLimits

#Vision System files:

from picamera.array import PiRGBArray
from picamera import PiCamera
import cv2
import numpy as np

from VisionSystemModule import VisionSystem


# ***** MAIN ***** #
if __name__ == "__main__":

    #START STATE

    #####
    print('STARTING ROBOT ARM...')

    #Initialise the Pi hat
    bus = smbus.SMBus(1) # the chip is on bus 1 of the available I2C buses
    addr = 0x40  # I2C address of the PWM chip.
    initialise_piHat()
    
    #Initialises the whole robot arm
    (m1,m2,m3,m4,m5) = Initialise_Robot_arm(bus)
    
    #Start arm at home position
    Move_arm_to_home(m1,m2,m3,m4,m5,bus)

    ####DEFINITIONS####
    #Measure arm segments l1,l2,l3,l4
    arm_lengths = (100,120,140,140)

    #Determine initial position set the desired pose there
    joint_values = read_arm(m1,m2,m3,m4)
    #initial conditions (0.0, 135.0, -100.0, -90.0)

    #Speed limit for velocity control:
    speedlimit = 10 # mm per dt

    #set Values for gripper:
    closed_grip_value = 410
    open_grip_value = 900    

    #################-------------------VISION SYSTEM SEE STATE--------------------################

    #Run Function
    coordinate = VisionSystem()
    print('Saw target now moving')
    time.sleep(5)
    
    #Target Point
    Desired_pose = (coordinate[0],coordinate[1],0,-pi/2)
    #Desired_pose = (175,50,0,-pi/2)
    
    #################-------------------MOVE AND PICK STATE--------------------####################
    print('Moving to target')
    moving = True

    #Run loop to go to the desired pose
    while(moving):

        #Compute the Current tool point
        tool_point = ForwardKinematics(joint_values,arm_lengths)       

        #Check if the tool_point is at the desired pose:
        if measure_position_error(Desired_pose,tool_point)<2:
            #Success
            print('Target was reached')
            moving = False
            time.sleep(2)
            m5.move_pwm(closed_grip_value,bus)
            time.sleep(2)
            
        else:
            #Keep moving to target
            print('Moving ... ')
            #compute the error vector and normalise it to the speed limit
            dX = compute_error(Desired_pose,tool_point,speedlimit)

            #compute the Jacobian / derivative of tool and joint
            J = compute_Jacobian(joint_values,arm_lengths)
            
            #Retrieve joint limits:
            (joint_mins, joint_maxs) = robotarmLimits(m1,m2,m3,m4)
            
            #Compute the inverse Jacobian 
            inv_J = damped_least_squares(J, joint_values, joint_mins, joint_maxs)
            
            #Estimate the required joint update to reduce positon error
            dQ = numpy.dot(inv_J,dX)

            #update the joints
            new_joint_values = update_joints(joint_values,dQ)

            #Move the robot arm
            move_arm(new_joint_values,open_grip_value,m1,m2,m3,m4,m5,bus)

            #Read the current joint angles
            joint_values = read_arm(m1,m2,m3,m4)
                    
    
    #################-------------------MOVE AND DROP STATE--------------------####################
    #Target Point
    Desired_pose = (140,140,140,-pi/6)    
    print('Moving to Box')
    droping = True

    #Run loop to go to the desired pose
    while(droping):

        #Compute the Current tool point
        tool_point = ForwardKinematics(joint_values,arm_lengths)       

        #Check if the tool_point is at the desired pose:
        if measure_position_error(Desired_pose,tool_point)<2:
            #Success
            print('Dropping ball')
            droping = False
            time.sleep(2)
            m5.move_pwm(open_grip_value,bus)
            time.sleep(4)
            
        else:
            #Keep moving to target
            print('Moving ... ')
            #compute the error vector and normalise it to the speed limit
            dX = compute_error(Desired_pose,tool_point,speedlimit)

            #compute the Jacobian / derivative of tool and joint
            J = compute_Jacobian(joint_values,arm_lengths)
            
            #Retrieve joint limits:
            (joint_mins, joint_maxs) = robotarmLimits(m1,m2,m3,m4)
            
            #Compute the inverse Jacobian 
            inv_J = damped_least_squares(J, joint_values, joint_mins, joint_maxs)
            
            #Estimate the required joint update to reduce positon error
            dQ = numpy.dot(inv_J,dX)

            #update the joints
            new_joint_values = update_joints(joint_values,dQ)

            #Move the robot arm
            move_arm(new_joint_values,closed_grip_value,m1,m2,m3,m4,m5,bus)

            #Read the current joint angles
            joint_values = read_arm(m1,m2,m3,m4)

            
    #END STATE
    #When the time kills the while loop, reset the robot and turn off 
    Reset_robot_arm(m1,m2,m3,m4,m5,bus)
    print('ROBOT ARM PROGRAM HAS ENDED')
