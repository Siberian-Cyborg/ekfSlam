function [triangle, R] = BodyToGlobal(triangle, x)

%angle of rotation
theta_g = x(3);
%disp("theta \t%f\n", theta);
%rotational matrix
R=[cos(theta_g) -sin(theta_g);
    sin(theta_g) cos(theta_g)];
%triangle oriented in heading direction of car

triangle = R*triangle;

triangle(1,:) = triangle(1,:) + x(1);
triangle(2,:) = triangle(2,:) + x(2);


end

