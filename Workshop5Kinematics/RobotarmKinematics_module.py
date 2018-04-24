"""
Robot arm Kinematics Module Kinematics Workshop Version
Based on theory from Intro to Robotics
Created for Semester 1 workshops 2018
@author: Andrew Razjigaev President of QUT Robotics Club
"""
import numpy

from math import sin, cos, pi

#Kinematics Workshop Version - will get fatter in the next workshops

#TODO:
#Find the DH Parameters for Each Joint in the Forward Kinematics Function


def DH_matrix(theta,d,a,alpha):
    '''
    Computes the Homogeneous Transformation matrix A from 4 parameters:
    such that:
    A = Rz(theta)*Tz(d)*Tx(a)*Rx(alpha)    
    '''
    A = numpy.matrix([[cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta)],
                      [sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta)],
                      [0,           sin(alpha),             cos(alpha),            d],
                      [0,           0,                      0,                     1]])
    return A;
    
    
def ForwardKinematics(joint_values,arm_lengths):
    '''
    Computes the Forward Kinematics i.e. x,y,z and pitch angle of the tool 
    point given joint angles and lengths  
    '''
    q1,q2,q3,q4 = joint_values
    l1,l2,l3,l4 = arm_lengths
    
    a0 = 0 #odd offset is 0 only if rotating base z axis points 
           #to the origin of the joint 2 coordinate frame

    #ADD DH PARAMETERS IN HERE
    baseTjoint1 = DH_matrix()
    joint1Tjoint2 = DH_matrix()
    joint2Tjoint3 = DH_matrix()
    joint3Tgripper = DH_matrix()

    BT2 = numpy.dot(baseTjoint1,joint1Tjoint2)
    BT3 = numpy.dot(BT2,joint2Tjoint3)
    baseTgripper = numpy.dot(BT3,joint3Tgripper)
    
    x = baseTgripper[0,3]
    y = baseTgripper[1,3]
    z = baseTgripper[2,3]
    
    pitch = q2 + q3 + q4
    
    return (x,y,z,pitch);  

  


