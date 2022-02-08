function [x,P] = EKF_prediction1(x, P, q_dot, Q,dt)
%function [x,P]= EKF_prediction1(x, P, q_dot, Q,dt)
%
% Inputs:
%   x, P - SLAM state and covariance
%   Q - covariance matrix for command
%   dt - timestep
%   q_dot = [v_Bx; v_By; omega_B];

% Outputs:
%   x, P - predicted state and covariance
%


%change scaling of input command
q_dot(1) = q_dot(1)*0.01;
q_dot(2) = q_dot(2)*0.01;
q_dot(3) = q_dot(3)*0.8;

% use orientation of robot to calculate rotation matrix from platform to
% world coordinates

R=[cos(x(3)+ q_dot(3)*dt) -sin(x(3)+ q_dot(3)*dt);
    sin(x(3)+ q_dot(3)*dt) cos(x(3)+ q_dot(3)*dt)];

% rotate x,y velocity input into global coordinate frame
trans = R*[q_dot(1);q_dot(2)];

%update of robots pose with motion model
x(1:3)=[x(1)+ trans(1)*dt;
    x(2)+ trans(2)*dt;
    x(3)+ q_dot(3)*dt];

% making sure robots heading stays between -pi and pi
x(3) = pi_to_pi(x(3));

%Jacobian of motion model with regards to state x
Gv= [1 0 (-sin(x(3)+ q_dot(3)*dt)*q_dot(1) -cos(x(3)+ q_dot(3)*dt)*q_dot(2))*dt;
    0 1 (cos(x(3)+ q_dot(3)*dt)*q_dot(1) -sin(x(3)+ q_dot(3)*dt)*q_dot(2))*dt;
    0 0 1];
%Jacobian of motion model with regards to controls
Gu= [cos(x(3)+ q_dot(3)*dt)*dt, -sin(x(3)+q_dot(3)*dt)*dt, (-sin(x(3)+ q_dot(3)*dt)*q_dot(1)*dt -cos(x(3)+ q_dot(3)*dt)*q_dot(2)*dt)*dt;
    sin(x(3)+ q_dot(3)*dt)*dt, +cos(x(3)+q_dot(3)*dt)*dt, (cos(x(3)+ q_dot(3)*dt)*q_dot(1)*dt -sin(x(3)+ q_dot(3)*dt)*q_dot(2)*dt)*dt;
    0 0 dt];
% update covariance of the system
P(1:3,1:3)= Gv*P(1:3,1:3)*Gv' + Gu*Q*Gu';
P(1:3,4:end) = Gv*P(1:3,4:end);
P(4:end,1:3) = P(1:3,4:end)';
 
end

