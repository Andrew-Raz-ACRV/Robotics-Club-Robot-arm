"""
Robot Arm Control Template Kinematics Workshop
Created for Semester 1 workshops 2018
Template made by: Andrew Razjigaev President of QUT Robotics Club
"""

#IMPORT STUFF
import smbus, numpy, time

from math import sin, cos, pi

#The Module for Servo Control (Last week's Motor Control Workshop)
#Check it out to review of motor control
from RobotarmServo_module import pi_servo, initialise_piHat

#New Module of arm control commands
from RobotarmUtility_module import move_arm_deg, Move_arm_to_home, Reset_robot_arm, read_arm, Initialise_Robot_arm

#New Module of Kinematics functions
from RobotarmKinematics_module import DH_matrix, ForwardKinematics

"""
READ ME:
Today's Workshop you need to complete a few functions:
In the Utilities module complete:
-Initialise_Robot_arm
-move_arm_deg
-move_arm_rad

In the Kinematics Module Complete:
-ForwardKinematics
"""

# ***** MAIN ***** #
if __name__ == "__main__":

    print('STARTING ROBOT ARM...')
    
    ####INITIALISATION####
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
    
    #set gripper to initial closed pwm
    old_grip_value = 410
    
#YOUR TURN!!!!

    #Do some move_arm stuff, while loop etc...  
    print('Doing Something...')

    #new_joint_values =

    #move_arm_deg

    #Do Forward Kinematics
    #Gripper_pose = ForwardKinematics(joint_values,arm_lengths)

    #print(Gripper_pose)


    #reset the robot and turn off 
    Reset_robot_arm(m1,m2,m3,m4,m5,bus)
    print('ROBOT ARM PROGRAM HAS ENDED')

    
