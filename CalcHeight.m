function [H] = ClacHeight(P_Ground,P,T0,H0,inKelvin)
%CLACHEIGHT Summary of this function goes here
%   Detailed explanation goes here
    if inKelvin
    H = ((1-(P/P_Ground)^(1/5.255))*T0)/0.0065;
    else
    H = ((1-(P/P_Ground)^(1/5.255))*(T0+273.15))/0.0065;    
    end
end

