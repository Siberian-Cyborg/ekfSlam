function [x]= add_landmark(x,z,z_cart,idx)

    range= z(1); 
    bearing= z(2);

    % add landmark to state vector x
    % this is a transformation from polar (range-bearing) coordinates
    % in plattform frame to cartesian coordinates in global frame
    x(idx)= [x(1) + range*cos(x(3)+ bearing);
             x(2) + range*sin(x(3)+ bearing)];
         
         
%     %angle of rotation
%     theta_g=x(3);
% 
%     %rotational matrix
%     R=[cos(theta_g) -sin(theta_g);
%        sin(theta_g)  cos(theta_g)];
% 
%     z_cart = R*z_cart;
% 
%     % translated by x
%     z_cart(1) = z_cart(1) + x(1);
%     % translated by y
%     z_cart(2) = z_cart(2) + x(2);
% 
%     x(idx)=[z_cart(1);z_cart(2)];

end