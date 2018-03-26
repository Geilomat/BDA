% In this script the State Estimation from given Sensor Data or self produced should be
% compared to the ground truth.

clear all; close all;


%% Initalize the System

% x1 = Height, x2 = Velocity, x3 Acceloration, x4 Temp , x5 Pressure
%Tempteratur at start = 15 C° -> 288.15 K°
load('PressLookUp.mat');

A = [0 1 0 0 0 0;0 0 1 0 0 0; 0 0 0 0 0 0;0 PressLookUp(1,1) 0 0 0 PressLookUp(2,1);0 0 0 0 0 1;0 0 0 0 0 0];
B = [0;0;0;0;0;0];                %No direct input
C = [1 0 0 0 0 0;0 0 1 0 0 0;0 0 0 1 0 0;0 0 0 1 0 0;0 0 0 0 1 0];          %Output ist Height and Acceleration
D = [0;0];
G = [0;0;1;1;0;1];                %System noise only on acceloration



% Get the time vector from the ARIS simulation.
load('TimeFromHassan.mat');
TimeVec = t';
Tau =  TimeVec(end)/length(TimeVec);


 %Diskretierung der Systemmatritzen
Ad = expm(A*Tau);     
Gd = Ad * G;
Bd = Ad * B;

% Dummy vairable for Simualation
simin = zeros(1,length(TimeVec));


%% Observability test;
rank([C;C*Ad;C*Ad*Ad])

%% Read sensor Data
load('StateFromHassan.mat');

% Get the heigth vector from the ARIS simulation.
h = state(:,3)';

% plot it
figure('Name','Real Data')
plot(h,'b') %%Original bahn
legend('Real height in z');


%% Produce noisless sensor Data self

figure('Name','Real Data');
% get acceloration by differentiate height:
a = diff(diff(h)/Tau)/Tau;
% Ad zeros to maintain vector length
a = [a 0 0];

% get height of GPS by deleting engouh values so it becomes 5Hz sample rate
% and then ad Zero Order Hold to get static value
GPSTau = 1/5;
h_GPS = zeros(1, length(TimeVec)-20);

for k =  1:(length(h)/round(GPSTau/Tau))
    for t =  1:round(GPSTau/Tau)
        h_GPS((k-1)*round(GPSTau/Tau)+t) = h((k-1)*round(GPSTau/Tau)+1);
   end
end

h_GPS = [h_GPS ones(1,20)*h_GPS(end)];

% Get a Temperatur vector with dicreasing Tempratur depending on height
T0 = 15 + 273.15;
T = T0 - 0.00649*h;


% Get Barometric Data (pressure)
% Pressure Data Temp/Po are just assumptions !!!!
Po = 1013.25;    %Pressure at altitude 0
p = Po*(1-(0.0065*h)./T).^5.255;



plot(TimeVec,h);
hold on;
plot(TimeVec,h_GPS)
plot(TimeVec,a);
plot(TimeVec,T)
plot(TimeVec,p);
legend('Real height in z','GPS heigt in z','Real Acceloration','Assumed Temperature in Kelvin','Assumed Pressure');
hold off;

%% Add noise to sensor data

T_mes = awgn(T,80,'measured');
h_mes_GPS = awgn(h_GPS,100,'measured');
p_mes_1 = awgn(p,35,'measured');
p_mes_2 = awgn(p,40,'measured');
a_mes = awgn(a,80,'measured');

figure('Name','Noise Data');
hold on;
plot(h_mes_GPS);
plot(p_mes_1);
plot(p_mes_2);
plot(a_mes);
plot(T_mes);
legend('GPS','Pressure 1','Pressure 2','Acceloration Measured','Temperatur');
hold off;

%% State estiamation initialize

% NoiseMatrices
%Sensor noise
GPS = 2;
ACL = 0.1;
BM1 = 5;
BM2 = 3;
TRM = 0.01;
R = diag([GPS ACL BM1 BM2 TRM]);
%Static System noise:
HGT = 0;
SPE = 0;
ACEL = 100;
PRE = 0.1;
TMP = 0;
DTMP = 0.1;
Q = [HGT;SPE;ACEL;PRE;TMP;DTMP];           
%Dynamic Sytem noise:
HGT = ones(1,length(TimeVec))*0.1;
SPE = ones(1,length(TimeVec))*0.1;
ACEL = [100 100 100 50 30 ones(1,length(TimeVec)-5)*20];
PRE = ones(1,length(TimeVec))*0.1;
TMP = ones(1,length(TimeVec))*0;
DTMP = ones(1,length(TimeVec))*0.1;
Q_dyn = [HGT;SPE;ACEL;PRE;TMP;DTMP];  
Q_dyn_t = timeseries(Q_dyn,TimeVec);

% Initialize
u = zeros(1,length(TimeVec));   %Input vector is zero
y = [h_mes_GPS;a_mes;p_mes_1;p_mes_2;T_mes];                %Output are the measurements
y_t = timeseries(y,TimeVec);
x = [0;0;0;Po;T(1);0];                    %Is reality
P0 = eye(6);

%% Excecute Simulation Static

sim('RocketSE');

%% Excecute Simulation Unscentend



%% Loop
for k = 1:length(TimeVec)
    K = P*C'*pinv(C*P*C' + R);
    x = x + K*(y(:,k) - C*x);
    P = (eye(3)-K*C)*P;
    
    s(k)=x(1); v(k)=x(2); aes(k)=x(3);  %Save data from the Sensor fusion
    
    x = Ad*x + B*u(k);
    P = Ad*P*Ad' + Gd*Q*Gd';

end

%% Plot

figure('Name','Real flight vs estimation');
plot(TimeVec,h);
hold on;
grid on;
plot(X_estimatd.time,X_estimatd.signals.values(:,1));
plot(TimeVec,a)
plot(X_estimatd.time,X_estimatd.signals.values(:,3));
plot(TimeVec,p)
plot(X_estimatd.time,X_estimatd.signals.values(:,4));
legend('real Height','estiamted Height','real acceloration','estimated acceloration','real pressure','estimated pressure');
ylabel('height & accelaration');
xlabel('Time [s]');