function [Pres] = CalcPressure(P0,H0,H,T0,inKelvin,TempGrad)
%CALCPRESSURE Summary of this function goes here
%   Detailed explanation goes here
M = 0.02896;
g = 9.807;
R = 8.314;

if inKelvin
    Pres = P0 * ((1-TempGrad*(H-H0))/(T0))^((M*g)/(R*TempGrad));
else
    Pres = P0 * ((1-TempGrad*(H-H0))/(T0+273.15))^((M*g)/(R*TempGrad));
end
end

