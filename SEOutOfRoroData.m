close all; clear all; clc;

%%
load('RoroTestIMU.mat')
load('RoroTestPressure.mat')
TimeVec = RoroTestIMUSmall.time;%(1:30000)';
Acc = RoroTestIMUSmall.acc_z;%(1:30000)';
Phi = RoroTestIMUSmall.gyro_z;%(1:30000)';
AccWGyro_mes = Acc.*cos(Phi);

Tau = 0.001;%TimeVec(end)/length(TimeVec);
Height = cumsum(cumsum(Acc*Tau)*Tau);
HeightWithGyro = cumsum(cumsum(AccWGyro_mes * Tau)*Tau);

figure('Name','Acceloration and Height');
plot(TimeVec,Acc,TimeVec,Height);
grid on;
hold on;
plot(TimeVec,HeightWithGyro);
hold off;

figure('Name','Acceloration with/without Gyro calculated');
plot(TimeVec,Acc);
hold on;
plot(TimeVec,AccWGyro_mes);
hold off;
legend('Without Gyro','With Gyro');

Po = 1013.25;    %Pressure at altitude 0
%p = Po*(1-(0.0065*h)./T).^5.255;
HeightPressure = (1+(RoroTestPress.static_pressure./Po).^(1/5.255).*(RoroTestPress.air_temp+273.15))./0.0065;
HeightPressure = HeightPressure - HeightPressure(1);
TimeVecPress = RoroTestPress.time';
Temp_mes = RoroTestPress.air_temp';
P_mes = RoroTestPress.static_pressure/100';
figure('Name','Height Pressure')
plot(TimeVecPress,HeightPressure);

%% Initialize a new System

A = [0 1 0 0; 0 0 1 0; 0 0 0 0;-0.00649 0 0 0];
B = [0;0;0;0];
C = [1 0 0 0; 0 0 1 0; 0 0 0 1];                                     % Input is Height out of Pressure, Acceloration and TemP
D = [0];

%Diskretierung der Systemmatritzen
Ad = expm(A*Tau);     

Bd = Ad * B;

disp(['Size of A: ' num2str(size(A))]);

%% Observability test;
disp(['Rank: ' num2str(rank([C;C*Ad;C*Ad*Ad]))]);

%% Get to the neeeded Matrices:

%Sensor Noise:
MPRS = 2^32;
MACL = 0.3;
MTMP = 2^32;
R = diag([MPRS;MACL;MTMP]);

%Static System noise:
HGT = 0;
SPE = 0;
ACEL = 70;
TMP = 0;
DTMP = 0.1;
Q = diag([HGT;SPE;ACEL;TMP;DTMP]);

%Dynamic Sytem noise:
HGT = ones(1,length(TimeVec))*HGT;
SPE = ones(1,length(TimeVec))*SPE;
ACEL = [100 100 100 50 30 ones(1,length(TimeVec)-5)*20];
TMP = ones(1,length(TimeVec))*TMP;
DTMP = ones(1,length(TimeVec))*DTMP;
Q_dyn = [HGT;SPE;ACEL;TMP];  

%%
%Add all noise vectors into an noise matrix
Q_dyn_m = diag(Q_dyn(:,1)');
for n = 2:length(TimeVec)
    Q_dyn_m = cat(3,Q_dyn_m,diag(Q_dyn(:,n)'));
end
%Q_dyn_t = timeseries(Q_dyn,TimeVec);

%% Initialize

u = zeros(1,length(TimeVec));                       %Input vector is zero
%y = [P_mes;AccWGyro_mes;Temp_mes];                  %Output are the measurements
%y_t = timeseries(y,TimeVec);
x0 = [0;0;0;Temp_mes(1)];                         %Start points
x =  x0;                                            %Is reality
P0 = eye(4);
P = eye(4);

%% Loop

x_est_loop = zeros(size(x,1),length(TimeVec));      %Vector for the SE values


for k = 1:length(TimeVec)
    %Calculate Tau an adjust A Matrix
    if k > 1
        Tau = TimeVec(k)-TimeVec(k-1);
        Ad = expm(A*Tau);     
    end
    
    %Detemine if a Press/Temp measurement is also avaiables
    index = find((TimeVec(k)-0.001)< TimeVecPress & TimeVecPress < (TimeVec(k) + 0.001));
    if index ~= 0;  
        K = P*C'*pinv(C*P*C' + [0.1 0 0;0 MACL 0;0 0 0.1]);
        Height = CalcHeight(Po,P_mes(index),Temp_mes(index),0);
        Temp = Temp_mes(index);
    else
        K = P*C'*pinv(C*P*C' + R);
    end
    
    
    x = x + K*([Height;AccWGyro_mes(k);Temp] - C*x);
    P = (eye(4)-K*C)*P;
    
    x_est_loop(:,k) = x;                            %Save data from the Sensor fusion
    
    x = Ad*x + Bd*u(k);
    
    P = Ad*P*Ad' + Q_dyn_m(:,:,k); 

end