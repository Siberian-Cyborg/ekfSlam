function [T] = CameraPlatformTransform(p,phi)
% Transforms points form camera coordinatesystem to platform coordinate
% system
%pitch rotation
R1 = [cos(pi/2-phi) 0  sin(pi/2-phi);
       0            1     0    ;
     -sin(pi/2-phi)  0  cos(pi/2-phi)];

%Z Rotation
R2= [cos(pi/2) sin(pi/2) 0;
     -sin(pi/2) cos(pi/2) 0;
     0            0      1];

R = R1*R2;

T = [R          p;
     zeros(1,3)  1];

%das ist die Trafo wenn pitch Null ist
% T = [0  0 1 p(1);
%      -1 0 0 p(2);
%      0 -1 0 p(3);
%      0  0 0  1];
end

