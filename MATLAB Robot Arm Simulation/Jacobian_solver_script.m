function Jacobian_solver_script()
%This script computes the Jacobian matrix symbolically for the forward
%kinematics equations
close all
clc

%Compute forward kinematics symbolically
syms q1 q2 q3 q4 l1 l2 l3 l4
q = [q1 q2 q3 q4];
L = [l1 l2 l3 l4];
[X] = RobotArmForwardKinematics(q,L);

%Simplify as much as Matlab can and use vpa to round numbers to n decimals
n = 4;
x = vpa(simplify(X(1)),n);
y = vpa(simplify(X(2)),n);
z = vpa(simplify(X(3)),n);
p = vpa(simplify(X(4)),n);

disp('The Forward Kinematics Equations')
display(x); display(y); display(z); display(p);

%Now use the Matlab Jacobian function by differentiating X and q
disp('The Jacobian symbolically is:')
J = jacobian(X,q); 
J = vpa(simplify(J),n);
display(J); %Display symbolic Jacobian

end
