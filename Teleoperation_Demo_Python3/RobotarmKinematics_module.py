"""
Robot arm Kinematics Module
Based on theory from Intro to Robotics
Created for Semester 1 workshops 2018
@author: Andrew Razjigaev President of QUT Robotics Club
"""
import numpy

from math import sin, cos, pi

def update_joints(joint_values,dQ):
    '''
    Does simple rectangular integration to update joints
    '''
    (q1,q2,q3,q4) = joint_values
    
    q1 = q1 + dQ[0,0]
    q2 = q2 + dQ[1,0]
    q3 = q3 + dQ[2,0]
    q4 = q4 + dQ[3,0]
    
    return (q1,q2,q3,q4);

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
    
    baseTjoint1 = DH_matrix(q1,l1,0,pi/2)
    joint1Tjoint2 = DH_matrix(q2,0,l2,0)
    joint2Tjoint3 = DH_matrix(q3,0,l3,0)
    joint3Tgripper = DH_matrix(q4,0,l4,0)

    BT2 = numpy.dot(baseTjoint1,joint1Tjoint2)
    BT3 = numpy.dot(BT2,joint2Tjoint3)
    baseTgripper = numpy.dot(BT3,joint3Tgripper)
    
    x = baseTgripper[0,3]
    y = baseTgripper[1,3]
    z = baseTgripper[2,3]
    
    pitch = q2 + q3 + q4
    
    return (x,y,z,pitch);  

def measure_position_error(Desired_pose,tool_point):
    '''
    Computes the cartesian distance to get to Desired_pose from tool_point
    as a scalar
    '''
    (X,Y,Z,P) = Desired_pose
    (x,y,z,p) = tool_point    
    error = numpy.matrix([[X-x], [Y-y], [Z-z], [P-p]])
    magnitude = numpy.linalg.norm(error)

    return magnitude;
  

def compute_error(Desired_pose,tool_point,speedlimit):
    '''
    Computes the error column vector dX
    dX = Desired tool position - current tool position
    With a speed limit
    ''' 
    (X,Y,Z,P) = Desired_pose
    (x,y,z,p) = tool_point
    
    error = numpy.matrix([[X-x], [Y-y], [Z-z], [P-p]])
    magnitude = numpy.linalg.norm(error)
    
    if magnitude>speedlimit:
        #Normalise and adjust magnitude of error vector to speed limit
        error = speedlimit * (error / magnitude)
        
    return error; 



def compute_Jacobian(joint_values,arm_lengths):
    '''
    Computes the current Jacobian matrix given joint values and arm lengths in 
    tuples
    All equations precomputed from MATLAB via the symbolic version of the 
    forward kinematics and the Jacobian function
    '''
    (q1,q2,q3,q4) = joint_values
    (l1,l2,l3,l4) = arm_lengths
    
    #Content from MATLAB files Jacobian_solver_script.m using the symbolic toolbox

    J = numpy.identity(4) #initialise the size

    J[0,0] = 0.5*l4*sin(q2 - 1.0*q1 + q3 + q4) - 0.5*l2*sin(q1 - 1.0*q2) - 0.5*l4*sin(q1 + q2 + q3 + q4) - 0.5*l2*sin(q1 + q2) + 0.5*l3*sin(q2 - 1.0*q1 + q3) - 0.5*l3*sin(q1 + q2 + q3)
    J[0,1] = 0.5*l2*sin(q1 - 1.0*q2) - 0.5*l4*sin(q2 - 1.0*q1 + q3 + q4) - 0.5*l4*sin(q1 + q2 + q3 + q4) - 0.5*l2*sin(q1 + q2) - 0.5*l3*sin(q2 - 1.0*q1 + q3) - 0.5*l3*sin(q1 + q2 + q3)
    J[0,2] = - 0.5*l4*sin(q2 - 1.0*q1 + q3 + q4) - 0.5*l4*sin(q1 + q2 + q3 + q4) - 0.5*l3*sin(q2 - 1.0*q1 + q3) - 0.5*l3*sin(q1 + q2 + q3)
    J[0,3] = - 0.5*l4*sin(q2 - 1.0*q1 + q3 + q4) - 0.5*l4*sin(q1 + q2 + q3 + q4)
    
    J[1,0] = 0.5*l2*cos(q1 - 1.0*q2) + 0.5*l4*cos(q1 + q2 + q3 + q4) + 0.5*l2*cos(q1 + q2) + 0.5*l3*cos(q2 - 1.0*q1 + q3) + 0.5*l3*cos(q1 + q2 + q3) + 0.5*l4*cos(q2 - 1.0*q1 + q3 + q4)
    J[1,1] = 0.5*l4*cos(q1 + q2 + q3 + q4) - 0.5*l2*cos(q1 - 1.0*q2) + 0.5*l2*cos(q1 + q2) - 0.5*l3*cos(q2 - 1.0*q1 + q3) + 0.5*l3*cos(q1 + q2 + q3) - 0.5*l4*cos(q2 - 1.0*q1 + q3 + q4)
    J[1,2] = 0.5*l4*cos(q1 + q2 + q3 + q4) - 0.5*l3*cos(q2 - 1.0*q1 + q3) + 0.5*l3*cos(q1 + q2 + q3) - 0.5*l4*cos(q2 - 1.0*q1 + q3 + q4)
    J[1,3] = 0.5*l4*cos(q1 + q2 + q3 + q4) - 0.5*l4*cos(q2 - 1.0*q1 + q3 + q4)
        
    J[2,0] = 0
    J[2,1] = l3*cos(q2 + q3) + l2*cos(q2) + l4*cos(q2 + q3 + q4)
    J[2,2] = l3*cos(q2 + q3) + l4*cos(q2 + q3 + q4)
    J[2,3] = l4*cos(q2 + q3 + q4)
     
    J[3,0] = 0
    J[3,1] = 1
    J[3,2] = 1
    J[3,3] = 1
    
    return J;
 
    

def damped_least_squares(J, Q, Qmin, Qmax): 
    '''
    Computes an inverse to the Jacobian such that it ensure the controller 
    avoids joint limits and minimises joint motion towards target
    therefore optimising the controller
    '''
    c = 1; p = 2; w = 1;

    Lambda = numpy.identity(4)
    
    #Construct the the dampening matrix Lambda
    for ii in range(0,4):
        num = 2*Q[ii]-Qmax[ii]-Qmin[ii];
        den = Qmax[ii] - Qmin[ii];
        Lambda[ii,ii] = c*(num/den)**p + w;

    #Compute the damped least squares inverse
    DL = numpy.dot(Lambda,Lambda)
    dampening = numpy.dot(J,J.T)
    dampening = dampening + DL
    
    invJ = numpy.dot(J.T,numpy.linalg.inv(dampening));
    
    #Try this solution the original inverse of the jacobian to compare performance
    #Notice the difference in motion planning?
    #invJ =numpy.linalg.inv(J)
    
    return invJ;


def WorkspaceBoundaryCheck(Q, Qnew, Qmin, Qmax):
    '''
    Logic that checks if the robot is stuck on a workspace boundary
    true if it is false if not
    '''

    #for each joint
    for ii in range(0,4):

        #Test if joint is close to the minimum boundary
        if numpy.abs(Q[ii]-Qmin[ii])<numpy.radians(2):
            
            #And if it will return to workspace in the next update:
            if (Q[ii]+Qnew[ii])>Qmin[ii]:
                result = False
            else:
                result = True  

            #Test if joint is close to the maximum boundary
        elif numpy.abs(Q[ii]-Qmax[ii])<numpy.radians(2):

            #And if it will return to workspace in the next update:
            if (Q[ii]+Qnew[ii])<Qmax[ii]:
                result = False
            else:
                result = True
        else:
            result = False

    return result;
    
