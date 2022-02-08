function [T] = PlatformMecanumTransform(R,p,g,a)
%Transfromation from Platfrom coordinatesystem to System of Mecanum wheels

M1 = [1 0 -p(2,1);
      0 1 p(1,1)];

M2 = [1 0 -p(2,2);
      0 1 p(1,2)];

M3 = [1 0 -p(2,3);
      0 1 p(1,3)];

M4 = [1 0 -p(2,4);
      0 1 p(1,4)];
  
R1 = [cos(g(1)) -sin(g(1));
      sin(g(1)) cos(g(1))];
  
R2 = [cos(g(2)) -sin(g(2));
      sin(g(2)) cos(g(2))];
  
R3 = [cos(g(3)) -sin(g(3));
      sin(g(3)) cos(g(3))];
  
R4 = [cos(g(4)) -sin(g(4));
      sin(g(4)) cos(g(4))];
  
K1 = [1 -cot(a(1))];

K2 = [1 -cot(a(2))];

K3 = [1 -cot(a(3))];

K4 = [1 -cot(a(4))];

T1 = 1/R * K1 * R1 * M1;
T2 = 1/R * K2 * R2 * M2;
T3 = 1/R * K3 * R3 * M3;
T4 = 1/R * K4 * R4 * M4;

T = [T1; 
     T2; 
     T3; 
     T4];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

end

