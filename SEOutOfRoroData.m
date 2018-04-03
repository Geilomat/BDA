close all; clear all; clc;

%%

%load('RoroTestIMU.mat')
TimeVec = RoroTestIMUSmall.time';
Acc = RoroTestIMUSmall.acc_z';
Acc(:) = Acc.*cos(RoroTestIMUSmall.gyro_z');

Tau = TimeVec(end)/length(TimeVec);
Height = cumsum(cumsum(Acc*Tau)*Tau);


plot(TimeVec,Acc,TimeVec,Height);
