function [p,pitch] = CamPose()

%in m
x = 0.2411;
y = 0;
z = 0.125;
p = [x;y;z];
%in deg
pitch = 5;
%transfrom to rad
pitch = pitch * pi/180;
end

