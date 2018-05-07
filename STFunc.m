function [xnew] = STFunc(x,w,Tau,Us1)
%STFUNC Summary of this function goes here
%   Detailed explanation goes here
xnew(1) = x(2) ;%* Tau;       %Height
xnew(2) = x(3) ;%* Tau;       %Speed
xnew(3) = 0;%x(3) + w(3);      %Acceleration
xnew(4) = 0;%x(4) + w(4);      %Acceleration Offset
xnew(5) = 0;%x(5) + w(5);      %Phi
xnew(6) = 0;%x(6) + w(6);      %Temperature
end

