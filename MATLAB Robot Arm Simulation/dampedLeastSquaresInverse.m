function [inv_J] = dampedLeastSquaresInverse(J,Joint_Values,Lower_Joint_Limits,Upper_Joint_Limits)
% This calculates the inverse of a Jacobian based on the damped least
% algorithms with embedded joint limit avoidance. 
%
% J^-1 = J'*(J*J' + D(lambda)^2)^-1
%
% Given a Jacobian matrix and the Joint limits in form (q1,q2,q3,q5)'
% the function computes the pseudo inverse of the Jacobian such that the
% joint values are minimised and the limits are avoided as much as possible
% on the path planning.
%
% Written by Andrew Razjigaev
%
% Based on Paper: 
% M. Na, B. Yang and P. Jia, “Improved damped least squares solution with 
% joint limits, joint weights and comfortable criteria for controlling 
% human-like figures”, IEEE Conference on Robotics, Automation and 
% Mechatronics, 2008: 1090-1095.

%First compute the Damping Matrix D(lambda) with Lambda being the joint
%penalties for joint limits
D = eye(4);
%General gains
c = 1; p = 2; w = 1;

for ii = 1:4
    qmin = Lower_Joint_Limits(ii);
    qmax = Upper_Joint_Limits(ii);
    qi = Joint_Values(ii);
    
    num = 2*qi - qmax - qmin;
    den = qmax - qmin;
    
    lambda = c*((num)/(den))^p + (1/w);
    
    D(ii,ii) = lambda;
end

%Now compute the Jacobian pseudo inverse by the formula and by the Jacobian
%Dimensions for either left or right inverse
if (size(J,1)<size(J,2))
    inv_J = (J'*J + D^2)\J';
else
    inv_J = J'/(J*J' + D^2);
end

end
