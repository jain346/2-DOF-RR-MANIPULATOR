function [omega1,omega2] = AngularCalc(theta1,theta2, theta1prev, theta2prev)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

omega1 = (theta1 - theta1prev)/ 0.05;
omega2 = (theta2- theta2prev) / 0.05;

end
