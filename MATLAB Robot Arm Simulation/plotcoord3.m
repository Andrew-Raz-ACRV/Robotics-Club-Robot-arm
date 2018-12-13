function plotcoord3(transform,axis_length,X_colour,Y_colour,Z_colour)
%Plots a 3D coordinate frame given a transform, the axis length and colours
%for each axis
%e.g. 
% R = eye(3); t = [0 0 0]';
%plotcoord3([R, t; 0 0 0 1],1,'r','g','b')
%
%this plots a coordinate frame at the origin with a red x-axis, green
%y-axis and a blue z-axis
%
%Tips: other useful functions related to this after plotting...
%
% title('3D View (X-Y-Z)'), xlabel('X displacement')
% ylabel('Y displacement'), zlabel('Z displacement')
% view(3)
% daspect([1 1 1]) % makes axes equal in 3D
% grid on
%
%written by Andrew Razjigaev

    %Axis Lengths of the axes on the robot coordinate frame
    px = [axis_length; 0; 0; 1]; py = [0; axis_length; 0; 1]; pz = [0; 0; axis_length; 1];
    %Homogeneous transform of the axes
    x = transform(1,4); y = transform(2,4); z = transform(3,4);
    %Computed points of axes:
    px_point = transform*px;
    py_point = transform*py;
    pz_point = transform*pz;
    %disp(transform)
    
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