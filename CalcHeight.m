function [H] = ClacHeight(P_Ground,P,T0,H0,inKelvin,TempGrad)
%CLACHEIGHT Summary of this function goes here
%   Detailed explanation goes here
M = 0.02896;
g = 9.807;
R = 8.314;

    if inKelvin
    H = ((1-(P/P_Ground)^((R*TempGrad)/(M*g)))*T0)/TempGrad;
    else
    H = ((1-(P/P_Ground)^((R*TempGrad)/(M*g)))*(T0+273.15))/TempGrad;    
    end
end

