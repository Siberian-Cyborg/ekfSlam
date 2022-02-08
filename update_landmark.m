function [x,P] = update_landmark(x, P, z, R, idx)

    dx = x(idx(1)) - x(1); %delta x between robot and tag
    dy = x(idx(2)) - x(2); %delta y between robot and tag
    q = dx*dx + dy*dy; %squared distance

    % calculating expected location of tag with observation model
    % observation model here means transformation from cartesian coordinates
    % in world frame to polar (range-bearing) coordinates in plattform frame
    z_expected = [sqrt(q); 
                  pi_to_pi(atan2(dy,dx)-x(3))];

    %difference between expected and true location of tag
    d = z - z_expected;
    %normalize angle
    d(2) = pi_to_pi(d(2));

    %this is a helper matrix to transform from observation space
    %to state space
    F = zeros(5, size(P,2));
    F(1:3,1:3)= eye(3);
    F(4:5,idx) = eye(2);

    %jacobian of the observation model
    Hi = (1/q) * [-sqrt(q)*dx, -sqrt(q)*dy, 0, sqrt(q)*dx, sqrt(q)*dy;
                       dy,         -dx,    -q,     -dy,       dx     ];
    %mapping jacobian to higher dimesional state space
    H =  Hi * F;
    %calculating Kalman gain
    K = (P * H') * inv(H*P*H'+ R);

    %update the state vector and the covariance matrix
    x = x + K*d;
    P = (eye(size(P,1)) - K*H)*P;
    
    %normalize heading of robot
    x(3) = pi_to_pi(x(3));
    
end