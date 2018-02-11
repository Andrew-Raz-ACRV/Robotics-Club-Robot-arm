function Jacobian_solver()

clc

%Compute forward kinematics symbolically
syms q1; syms q2; syms q3; syms q4; syms lo; syms l1; syms l2; syms l3;

[x,y,z,a] = forwardkinematics(q1,q2,q3,q4,lo,l1,l2,l3);

%Simplify as much as Matlab can...
x = simplify(x);
y = simplify(y);
z = simplify(z);
a = simplify(a);

disp('The Forward Kinematics Equations')
display(x)
display(y)
display(z)
display(a)

disp("Don't worry about the big numbers we can simplify that ourselves...")
disp('The numbers 81129638414606676728031405122553/162259276829213363391578010288128 should equate to 1/2')

disp('copy the equations into the functions F1, F2, F3, F4')

%Use the equations as functions for the inputs to the Jacobian function
Q = [q1; q2; q3; q4];

F1 = f1(Q); F2 = f2(Q); F3 = f3(Q); F4 = f4(Q);

F = [F1; F2; F3; F4];

%Now use the Matlab Jacobian function
disp('The Jacobian symbolically is:')
J = jacobian(F,Q); 
J = simplify(J);
display(J); %Display symbolic Jacobian

end

%Forward kinematics functions for the solver
function [X] = f1(Q)

syms lo; syms l1; syms l2; syms l3;

q1 = Q(1); q2 = Q(2); q3 = Q(3); q4 = Q(4);

X = 0.5*l2*cos(q2 - q1 + q3) + 0.5*l3*cos(q1 + q2 + q3 + q4) ...
    + 0.5*cos(q1 + q2) + 0.5*l3*cos(q2 - q1 + q3 + q4) + ...
    0.5*l1*cos(q1 - q2) + 0.5*l2*cos(q1 + q2 + q3);

end

function [Y] = f2(Q)

syms lo; syms l1; syms l2; syms l3;

q1 = Q(1); q2 = Q(2); q3 = Q(3); q4 = Q(4);

Y = 0.5*l3*sin(q1 + q2 + q3 + q4) - 0.5*l2*sin(q2 - q1 + q3)...
    + 0.5*l1*sin(q1 + q2) - 0.5*l3*sin(q2 - q1 + q3 + q4)...
    + 0.5*l1*sin(q1 - q2) + 0.5*l2*sin(q1 + q2 + q3);
 
end

function [Z] = f3(Q)

syms lo; syms l1; syms l2; syms l3;

q2 = Q(2); q3 = Q(3); q4 = Q(4);

Z = lo - l2*sin(q2 + q3) - l1*sin(q2) - l3*sin(q2 + q3 + q4);

end

function [A] = f4(Q)

syms lo; syms l1; syms l2; syms l3;

q2 = Q(2); q3 = Q(3); q4 = Q(4);

A = q2 + q3 + q4;

end


%*********FORWARD KINEMATICS*******%
function [x,y,z,a] = forwardkinematics(q1,q2,q3,q4,lo,l1,l2,l3)
    
    %Compute Homogeneous Transformations
    Base_T_j0 = DH_params(q1,lo,0,-pi/2);

    j0_T_j1 = DH_params(q2,0,l1,0);

    j1_T_j2 = DH_params(q3,0,l2,0);
    
    j2_T_end = DH_params(q4,0,l3,0);

    Base_T_end = Base_T_j0 * j0_T_j1 * j1_T_j2 * j2_T_end;
    
    %Compute Forward Kinematics:
    x = Base_T_end(1,4);
    y = Base_T_end(2,4);
    z = Base_T_end(3,4);
    a = sum([q2 q3 q4]);

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