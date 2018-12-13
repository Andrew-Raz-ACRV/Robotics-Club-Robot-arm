function PlotRobotArm(q,L)
%Plots the current configuration of the robot arm given q = [q1,q2,q3,q4]
%and the length of the robot arm joints L = [L1, L2, L3, L4]
  
%Plot Base frame:
ax = 25; %mm axis length of coordinate frame
w = 5; %width of joints
plotcoord3(eye(4),ax,'r','g','b');

%Plot First joint:
BaseTj1 = DH(q(1),L(1),0,pi/2);
plotcoord3(BaseTj1,ax,'r','g','b');
%Plot connection
plot3([0 BaseTj1(1,4)],[0 BaseTj1(2,4)],[0 BaseTj1(3,4)],'k-','Linewidth',w);

%Plot Second joint:
j1Tj2 = DH(q(2),0,L(2),0);
BaseTj2 = BaseTj1*j1Tj2;
plotcoord3(BaseTj2,ax,'r','g','b');
%Plot connection
plot3([BaseTj1(1,4) BaseTj2(1,4)],[BaseTj1(2,4) BaseTj2(2,4)],[BaseTj1(3,4) BaseTj2(3,4)],'k-','Linewidth',w);

%Plot Third joint:
j2Tj3 = DH(q(3),0,L(3),0);
BaseTj3 = BaseTj2*j2Tj3;
plotcoord3(BaseTj3,ax,'r','g','b');
%Plot connection
plot3([BaseTj2(1,4) BaseTj3(1,4)],[BaseTj2(2,4) BaseTj3(2,4)],[BaseTj2(3,4) BaseTj3(3,4)],'k-','Linewidth',w);

%Plot Fourth joint:
j3Tgripper = DH(q(4),0,L(4),0);
BaseTg = BaseTj3*j3Tgripper;
plotcoord3(BaseTg,ax,'r','g','b');
%Plot connection
plot3([BaseTj3(1,4) BaseTg(1,4)],[BaseTj3(2,4) BaseTg(2,4)],[BaseTj3(3,4) BaseTg(3,4)],'k-','Linewidth',w);

%Tidy view:
axis equal
grid on
view([45,30])   
 
%Title captions:
title('Simulation of QUTRC Robot Arm'), xlabel('X axis (mm)')
ylabel('Y axis (mm)'), zlabel('Z axis (mm)')

end
