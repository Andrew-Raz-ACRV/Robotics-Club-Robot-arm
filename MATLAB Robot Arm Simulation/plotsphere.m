function plotsphere(centroid,radius,colour)
%Given a centroid = [x,y,z] and radius = positive scalar and a colour. 
%This function plots the sphere at that point with that radius and colour

%create sphere object
[X,Y,Z] = sphere(); %square matrices

%default radius is 1 so make it radius
for i = 1:size(X,1)
    for j = 1:size(X,2)
        %Get point
        x = X(i,j); y = Y(i,j); z = Z(i,j);
        %Set radius:
        x_ = cap_mag([x y z],radius);
        %Return
        X(i,j) = x_(1); 
        Y(i,j) = x_(2); 
        Z(i,j) = x_(3);  
        
    end
end
%Set centroid
X = centroid(1) + X;
Y = centroid(2) + Y;
Z = centroid(3) + Z;

%plot sphere
surf(X,Y,Z,'FaceColor',colour)
end