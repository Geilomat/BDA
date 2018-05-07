function [y] = SUFunc(x)%Us1)
%STFUNC Summary of this function goes here
%   Detailed explanation goes here
y(1) = x(1);                                             %GPS
y(2) = (x(3) + x(4))/cos(x(5));                          %Acc
y(3) = x(5);                                             %Phi
y(4) = 103.13*(1-(0.0065*(x(1)-1))/(x(6)+273.15))^5.255; %P1
y(5) = 103.13*(1-(0.0065*(x(1)-1))/(x(6)+273.15))^5.255; %P2
y(6) = x(6);                                             %T
end