function [x,P] = EKF_prediction2(x, P, wi, Q_wheels, dt, T_PM)
 %function [x,P]= EKF_prediction2(x, P, q_dot, Q,dt)
%
% Inputs:
%   x, P - SLAM state and covariance
%   Q_wheels - covariance matrix for command
%   dt - timestep
%   wi 4x1 vector with rad/s for each wheel
%   T_PM = 3x4 transformation matrix from base to wheel frame

% Outputs: 
%   x, P - predicted state and covariance

 alpha=pinv(T_PM)*wi';
 
 alpha(1) = alpha(1)*0.01;
 alpha(2) = alpha(2)*0.01;
 alpha(3) = alpha(3)*0.8;
 
 R=[cos(alpha(3)+ x(3)*dt) -sin(alpha(3)+ x(3)*dt) 0;
   sin(alpha(3)+ x(3)*dt) cos(alpha(3)+ x(3)*dt) 0;
   0,                 0,                            1];

alpha = R*alpha;

 x(1:3)=[x(1)+alpha(1)*dt;
     x(2)+alpha(2)*dt;
     pi_to_pi(x(3)+alpha(3)*dt)];
 
 Gv= [1 0 (-sin(alpha(3)+ x(3)*dt)*dt-cos(alpha(3)+ x(3)*dt)*dt)*dt;
     0 1 (cos(alpha(3)+ x(3)*dt)*dt-sin(alpha(3)+ x(3)*dt)*dt)*dt;
     0 0 1];
        
        %For update through module feedback

    %     T_PM=[cos(g1)-cot(alpha1)*sin(g1) -sin(g1)-cot(alpha1)*cos(g1) -py1*cos(g1)-px1*sin(g1)-cot(alpha1)*(-sin(g1)*py1+cos(g1)*py1);
    %       cos(g2)-cot(alpha2)*sin(g2) -sin(g2)-cot(alpha2)*cos(g2) -py2*cos(g2)-px2*sin(g2)-cot(alpha2)*(-sin(g2)*py2+cos(g2)*py2);
    %       cos(g3)-cot(alpha3)*sin(g3) -sin(g3)-cot(alpha3)*cos(g3) -py3*cos(g3)-px3*sin(g3)-cot(alpha3)*(-sin(g3)*py3+cos(g3)*py3);
    %       cos(g4)-cot(alpha4)*sin(g4) -sin(g4)-cot(alpha4)*cos(g4) -py4*cos(g4)-px4*sin(g4)-cot(alpha4)*(-sin(g4)*py4+cos(g4)*py4)]
    %   pinvT=pinv(T_PM)

    pinvT= [1.25 1.25 -1.25 -1.25;
        -1.25 1.25 -1.25 1.25;
        -0.0366 -0.0366 -0.0366 -0.0366];
    
    Gu=R*pinvT*dt;
    
    P(1:3,1:3)= Gv*P(1:3,1:3)*Gv' + Gu*Q_wheels*Gu';
    P(1:3,4:end) = Gv*P(1:3,4:end);
    P(4:end,1:3) = P(1:3,4:end)';

    
end

