function [theta1_1, theta2_1 , theta1_2,theta2_2] = R2Ikin(a1,a2, px,py)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
c2 = (px^2 + py^2 - a1^2 - a2^2)/ (2*a1*a2);

if c2 <= 1
    s2_1 = + sqrt(1- c2^2); s2_2 = - sqrt(1- c2^2);
    theta2_1 = atan2(c2,s2_1); %first solution of theta2
    theta2_2 = atan2(c2,s2_2); % second solution of theta2
    d1 = a1^2 + a2^2 + 2*a1*a2*cos(theta2_1);
    d2 = a1^2 + a2^2 + 2*a1*a2*cos(theta2_2);
    s1_1 = (py*(a1 + a2*cos(theta2_1)) - px*a2*sin(theta2_1))/ d1;
    s1_2= (py*(a1 + a2*cos(theta2_2)) - px*a2*sin(theta2_2))/ d2;
    c1_1 =(px*(a1 + a2*cos(theta2_1)) - py*a2*sin(theta2_1))/ d1;
    c1_2 = (px*(a1 + a2*cos(theta2_2)) - py*a2*sin(theta2_2))/ d2;

    theta1_1 = atan2(c1_1, s1_1);  %first solution of theta1
    theta1_2 = atan2(c1_2,s1_2);   % second solution of theta2

end
