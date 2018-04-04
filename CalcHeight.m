function [H] = ClacHeight(P_Ground,P,T,H0)
%CLACHEIGHT Summary of this function goes here
%   Detailed explanation goes here
    H = (1-(P/P_Ground)^5.255*(T+273.15))/0.0065;
end

