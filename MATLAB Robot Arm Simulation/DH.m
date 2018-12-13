function A = DH(theta,d,a,alpha)
% Computes the 3D transform matrix from frame j-1 to frame j
% using Denavit Hartenburg parameters theta, d, a, alpha
% i.e. A = Rz(theta)*Tz(d)*Tx(a)*Rx(alpha)

A = [cos(theta) -sin(theta)*cos(alpha) sin(theta)*sin(alpha) a*cos(theta);
     sin(theta) cos(theta)*cos(alpha) -cos(theta)*sin(alpha) a*sin(theta);
     0          sin(alpha)             cos(alpha)           d;
     0          0                       0                   1];

end