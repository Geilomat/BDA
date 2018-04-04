function [Pres] = CalcPressure(P0,H0,H,T)
%CALCPRESSURE Summary of this function goes here
%   Detailed explanation goes here
Pres = P0 * ((1-0.0065*H)/(T+273.15))^5.255;
end

