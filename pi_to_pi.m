function angle = pi_to_pi(angle)

%function angle = pi_to_pi(angle)
% Input: array of angles.


angle = mod(angle, 2*pi);

i=find(angle>pi);
angle(i)=angle(i)-2*pi;

i=find(angle<-pi);
angle(i)=angle(i)+2*pi;
end
