# -*- coding: utf-8 -*-
"""
Created on Sun Feb 11 22:42:02 2018

@author: Andrew
"""

import smbus, time, math, numpy

from ServoClass import Pi_servo





# ***** MAIN ***** #
if __name__ == "__main__":
    
    
    #initialise The I2C pi hat
    bus = smbus.SMBus(1)  # the chip is on bus 1 of the available I2C buses
    addr = 0x20           # I2C address of the PWM chip.
    bus.write_byte_data(addr, 0, 0x20)     # enable the chip
    bus.write_byte_data(addr, 0xfe, 0x1e)  # configure the chip for multi-byte write
    
    
    #initialise the servos
    m1 = Pi_servo()
    m2 = Pi_servo()
    m3 = Pi_servo()
    m4 = Pi_servo()
    m5 = Pi_servo()
    
    m1._init_channel(0)
    m2._init_channel(3)
    m3._init_channel(4)
    m4._init_channel(7)
    m5._init_channel(8)

    m1.start_servo(bus)
    m2.start_servo(bus)
    m3.start_servo(bus)
    m4.start_servo(bus)
    m5.start_servo(bus)

    #While loop for the control
    on = true;
    
    while(on):
        
        #Get input key
        key = raw_input()
        #Decide
        if key == 'k':
            on = false
            print('terminating robot arm')
        elif key == 'a':
            Y = Y + 5
        elif key == 'w':
            X = X + 5
        elif key == 's':
            X = X - 5
        elif key == 'd':
            Y = Y - 5
        elif key == 'u':
            Z = Z + 5
        elif key == 'j':
            Z = Z - 5
        elif key == 'e':
            #toggle open close

        
    

""" Matlab code for robot arm

function joint_command = RobotArmController(target,initial,arm_lengths)

%Interpret Input - target and initial and arm lengths
xt = target(1);
yt = target(2);
zt = target(3);
at = -deg2rad(target(4));

q0 = deg2rad(initial(1));
q1 = deg2rad(initial(2));
q2 = deg2rad(initial(3));
q3 = deg2rad(initial(4));

%Limits
lo = arm_lengths(1);
l1 = arm_lengths(2);
l2 = arm_lengths(3);
l3 = arm_lengths(4);

Qmax = deg2rad([45 45 45 45]);
Qmin = deg2rad([-45 -45 -45 -45]);

%Compute Inverse Kinematics

%Loop Solver
for ii=1:500
    
    %Compute Forward Kinematics from our function
    [x,y,z,a] = forwardkinematics(q0,q1,q2,q3,lo,l1,l2,l3);
    
    %Compute Velocity (Position error)
    dx = xt - x;
    dy = yt - y;
    dz = zt - z;
    da = at - a;
    
    dX = [dx dy dz da]';
    
    %Clamp magnitude
    max = 50;
    if norm(dX)>max
        dX = max*(dX)/norm(dX);
    end
    
    %Compute Jacobian from our function
    J = Jacobian_matrix(q0,q1,q2,q3,lo,l1,l2,l3);
     
    %Compute damped least squares inverse
    Q = [q0, q1, q2, q3];
    inv_J = damped_least(J, Q, Qmax, Qmin);
    
    %Compute output pseudo inverse Jacobian
    dQ = inv_J*dX;
    
    %Update joints
    q0 = q0 + dQ(1);
    q1 = q1 + dQ(2);
    q2 = q2 + dQ(3);
    q3 = q3 + dQ(4); 
    
    %Halt reading at joint limit
    if q0>Qmax(1)
        q0 = Qmax(1);
    end
    if q0<Qmin(1)
        q0 = Qmin(1);
    end
    
    if q1>Qmax(2)
        q1 = Qmax(2);
    end
    if q1<Qmin(1)
        q1 = Qmin(2);
    end
    
    if q2>Qmax(3)
        q2 = Qmax(3);
    end
    if q2<Qmin(3)
        q2 = Qmin(3);
    end
    
    if q3>Qmax(4)
        q3 = Qmax(4);
    end
    if q3<Qmin(4)
        q3 = Qmin(4);
    end
    
    
end


%Output solution
joint_command = [q0 -q1 -q2 -q3]';

%joint_command = deg2rad(initial);

%joint_command = deg2rad([30 30 30 30]');

end

function invJ = damped_least(J,Q,Qmax,Qmin)

c = 1; p = 2; w = 1;
%Compute Lambda
lambda = eye(size(J));

for ii = 1:length(Q)
    num = 2*Q(ii)-Qmax(ii)-Qmin(ii);
    den = Qmax(ii) - Qmin(ii);
    lambda(ii,ii) = c*(num/den)^p + w;
end

%Compute final solution
invJ = J'/(J*J'+lambda^2);

end

%*********DH PARAMETERS*********%
function frame1_T_frame2 = DH_params(theta,d,a,alpha)
% Computes the transformation matrix
% based on DH parameters
% A = Rz(theta)*Tz(d)*Tx(a)*Rx(alpha)

frame1_T_frame2 = [cos(theta) -sin(theta)*cos(alpha)  sin(theta)*cos(alpha) a*cos(theta);
                   sin(theta)  cos(theta)*cos(alpha) -cos(theta)*cos(alpha) a*sin(theta);
                   0           sin(alpha)             cos(alpha)            d;
                   0           0                      0                     1];

end

%*********FORWARD KINEMATICS*******%
function [x,y,z,a] = forwardkinematics(q0,q1,q2,q3,lo,l1,l2,l3)
    
    %Compute Homogeneous Transformations
    Base_T_j0 = DH_params(q0,lo,0,-pi/2);

    j0_T_j1 = DH_params(q1,0,l1,0);

    j1_T_j2 = DH_params(q2,0,l2,0);
    
    j2_T_end = DH_params(q3,0,l3,0);

    Base_T_end = Base_T_j0 * j0_T_j1 * j1_T_j2 * j2_T_end;
    
    %Compute Forward Kinematics:
    x = Base_T_end(1,4);
    y = Base_T_end(2,4);
    z = Base_T_end(3,4);
    a = sum([q1 q2 q3]);

end

%*************JACOBIAN**********%
function J = Jacobian_matrix(q1,q2,q3,q4,~,l1,l2,l3)

J = [(l2*sin(q2 - q1 + q3))/2 - sin(q1 + q2)/2 - (l3*sin(q1 + q2 + q3 + q4))/2 + (l3*sin(q2 - q1 + q3 + q4))/2 - (l1*sin(q1 - q2))/2 - (l2*sin(q1 + q2 + q3))/2, (l1*sin(q1 - q2))/2 - (l2*sin(q2 - q1 + q3))/2 - (l3*sin(q1 + q2 + q3 + q4))/2 - (l3*sin(q2 - q1 + q3 + q4))/2 - sin(q1 + q2)/2 - (l2*sin(q1 + q2 + q3))/2, - (l2*sin(q2 - q1 + q3))/2 - (l3*sin(q1 + q2 + q3 + q4))/2 - (l3*sin(q2 - q1 + q3 + q4))/2 - (l2*sin(q1 + q2 + q3))/2, -(l3*(sin(q1 + q2 + q3 + q4) + sin(q2 - q1 + q3 + q4)))/2;
                                                                                              cos(q1)*(l2*cos(q2 + q3) + l1*cos(q2) + l3*cos(q2 + q3 + q4)),                                                                                             -sin(q1)*(l2*sin(q2 + q3) + l1*sin(q2) + l3*sin(q2 + q3 + q4)),   (l3*cos(q1 + q2 + q3 + q4))/2 - (l2*cos(q2 - q1 + q3))/2 - (l3*cos(q2 - q1 + q3 + q4))/2 + (l2*cos(q1 + q2 + q3))/2,  (l3*(cos(q1 + q2 + q3 + q4) - cos(q2 - q1 + q3 + q4)))/2;
                                                                                                                                                         0,                                                                                                      - l2*cos(q2 + q3) - l1*cos(q2) - l3*cos(q2 + q3 + q4),                                                                              - l2*cos(q2 + q3) - l3*cos(q2 + q3 + q4),                                     -l3*cos(q2 + q3 + q4);
                                                                                                                                                          0,                                                                                                                                                          1,                                                                                                                     1,                                                         1];
 
end
"""
