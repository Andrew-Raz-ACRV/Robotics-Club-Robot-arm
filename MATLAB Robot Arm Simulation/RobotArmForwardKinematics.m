function [X] = RobotArmForwardKinematics(q,L)
% For the Armageddon Robotics club Robot arm, this computes the Forward
% kinematics X = [x,y,z,p]
% Given q = [q1,q2,q3,q4] and L = [l1,l2,l3,l4]

BaseTj1 = DH(q(1),L(1),0,pi/2);
j1Tj2 = DH(q(2),0,L(2),0);
j2Tj3 = DH(q(3),0,L(3),0);
j3Tgripper = DH(q(4),0,L(4),0);

BaseTgripper = BaseTj1*j1Tj2*j2Tj3*j3Tgripper;

%Extract toolpoint
x = BaseTgripper(1,4); y = BaseTgripper(2,4);
z = BaseTgripper(3,4);
%pitch angle
p = q(2)+q(3)+q(4);

X = [x,y,z,p];

end