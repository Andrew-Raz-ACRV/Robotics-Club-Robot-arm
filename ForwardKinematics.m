function ForwardKinematics()
clf
[x,y,z,~] = forwardkinematics(deg2rad(45),deg2rad(120),deg2rad(-100),deg2rad(-90));
    
display(x)
display(y)
display(z)

end

%*********DH PARAMETERS*********%
function frame1_T_frame2 = DH_params(theta,d,a,alpha)
% Computes the transformation matrix
% based on DH parameters
% Label coordinate frames as so:
% Z axis is the axis of rotation or prismatics actuation
% X axis is 

% A = Rz(theta)*Tz(d)*Tx(a)*Rx(alpha)

frame1_T_frame2 = [cos(theta) -sin(theta)*cos(alpha)  sin(theta)*sin(alpha) a*cos(theta);
                   sin(theta)  cos(theta)*cos(alpha) -cos(theta)*sin(alpha) a*sin(theta);
                   0           sin(alpha)             cos(alpha)            d;
                   0           0                      0                     1];

end

%*********FORWARD KINEMATICS*******%
function [x,y,z,a] = forwardkinematics(q0,q1,q2,q3)
    
    %The arm lengths
    lo = 50;
    l1 = 100;
    l2 = 100;
    l3 = 50;
    
    a_ = 0;
    
    %Compute Homogeneous Transformations
    Base_T_j0 = DH_params(q0,lo,a_,pi/2);

    j0_T_j1 = DH_params(q1,0,l1,0);

    j1_T_j2 = DH_params(q2,0,l2,0);
    
    j2_T_end = DH_params(q3,0,l3,0);

    Base_T_end = Base_T_j0 * j0_T_j1 * j1_T_j2 * j2_T_end;

    %Plot
    figure(1)
 
    ax = 10;
    plotcoord3(eye(4),ax,'r','g','b');
    plotcoord3(Base_T_j0,ax,'r','g','b');
        view(3)
    daspect([1 1 1])
    plotcoord3(Base_T_j0*j0_T_j1,ax,'r','g','b');
        view(3)
    daspect([1 1 1])
    plotcoord3(Base_T_j0*j0_T_j1*j1_T_j2,ax,'r','g','b');
        view(3)
    daspect([1 1 1])
    plotcoord3(Base_T_end,ax,'r','g','b');
    
    title('3D View (X-Y-Z)'), xlabel('X displacement')
    ylabel('Y displacement'), zlabel('Z displacement')
    view(3)
    daspect([1 1 1])
    
    %Compute Forward Kinematics:
    x = Base_T_end(1,4);
    y = Base_T_end(2,4);
    z = Base_T_end(3,4);
    a = 1;

end

%3D plot coordinate system function
function plotcoord3(transform,axis_length,X_colour,Y_colour,Z_colour)
    %Axis Lengths of the axes on the robot coordinate frame
    px = [axis_length; 0; 0; 1]; py = [0; axis_length; 0; 1]; pz = [0; 0; axis_length; 1];
    %Homogeneous transform of the axes
    x = transform(1,4); y = transform(2,4); z = transform(3,4);
    %Computed points of axes:
    px_point = transform*px;
    py_point = transform*py;
    pz_point = transform*pz;
    disp(transform)
    %X axis
    plot3([x; px_point(1)], [y; px_point(2)], [z; px_point(3)], X_colour)
    hold on 
    %Y axis
    plot3([x; py_point(1)], [y; py_point(2)], [z; py_point(3)], Y_colour)
    hold on  
    %Z axis
    plot3([x; pz_point(1)], [y; pz_point(2)], [z; pz_point(3)], Z_colour)
    hold on  
end
