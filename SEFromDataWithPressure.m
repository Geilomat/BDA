% In this script the State Estimation from given Sensor Data or self produced should be
% compared to the ground truth.

clear all; close all;


%% Initalize the System

% x1 = Height, x2 = Velocity, x3 Acceloration, x4 Temp , x5 Pressure
%Tempteratur at start = 15 C° -> 288.15 K°
load('PressLookUp.mat');

A = [0 1 0 0 0 0;0 0 1 0 0 0; 0 0 0 0 0 0;0 PressLookUp(1,1) 0 0 0 PressLookUp(2,1);0 0 0 0 0 1;0 0 0 0 0 0];
B = [0;0;0;0;0;0];                %No direct input
C = [0 0 1 0 0 0;0 0 0 1 0 0;0 0 0 1 0 0;0 0 0 0 1 0];          %Output ist Height and Acceleration
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
v = diff(h)/Tau;
a = diff(v)/Tau;
% Ad zeros to maintain vector length
v = [v 0];
a = [a 0 0];

% Get a Temperatur vector with dicreasing Tempratur depending on height
T0 = 15 + 273.15;
T = T0 - 0.00649*h;


% Get Barometric Data (pressure)
% Pressure Data Temp/Po are just assumptions !!!!
Po = 1013.25;    %Pressure at altitude 0
p = Po*(1-(0.0065*h)./T).^5.255;

%P1 Has less noise Therefore samples slower
P1Tau = 1/50;
P1 = zeros(1,length(TimeVec));
for k =  1:(length(p)/round(P1Tau/Tau))
    for t =  1:round(P1Tau/Tau)
        if (k-1)*round(P1Tau/Tau)+t <= length(TimeVec)
        P1((k-1)*round(P1Tau/Tau)+t) = p((k-1)*round(P1Tau/Tau)+1);
        end
   end
end
                            
%P2 Has more noise but samples faster
P2Tau = 1/100;
P2 = zeros(1,length(TimeVec));
for k =  1:(length(p)/round(P2Tau/Tau))
    for t =  1:round(P2Tau/Tau)
        if (k-1)*round(P2Tau/Tau)+t <= length(TimeVec)
        P2((k-1)*round(P2Tau/Tau)+t) = p((k-1)*round(P2Tau/Tau)+1);
        end
   end
end                           

plot(TimeVec,h);
hold on;
plot(TimeVec,a);
plot(TimeVec,T)
plot(TimeVec,P1);
plot(TimeVec,P2);
legend('Real height in z','Real Acceloration','Assumed Temperature in Kelvin','Assumed Pressure 1','Assumed Pressure 2');
hold off;

%% Add noise to sensor data

T_mes = awgn(T,40,'measured');
p_mes_1 = awgn(p,40,'measured');
p_mes_2 = awgn(p,35,'measured');
a_mes = awgn(a,30,'measured');

figure('Name','Noise Data');
hold on;
plot(p_mes_1);
plot(p_mes_2);
plot(a_mes);
plot(T_mes);
legend('Pressure 1','Pressure 2','Acceloration Measured','Temperatur');
hold off;

%% Create Noise Matrices

% Static Sensor noise
ACLvar = 0.1;
BM1var = 3;
BM2var = 1;
TRMvar = 0.01;
R = diag([ACLvar BM1var BM2var TRMvar]);

%Dynamic Sensor noise:
ACLvar = ones(1,length(TimeVec))*0.1;

BM1Step = [BM1var ones(1,round(P1Tau/Tau)-1)*2^32];
BM1var = [];
for n = 1:round(length(TimeVec)/length(BM1Step))-1
    BM1var = [BM1var BM1Step];
end
BM1var = [BM1var ones(1,length(TimeVec)-length(BM1var))*2^32];
%BM1var = ones(1,length(TimeVec))*0.5;

BM2Step = [BM2var ones(1,round(P2Tau/Tau)-1)*2^32];
BM2var = [];
for n = 1:round(length(TimeVec)/length(BM2Step))-1
    BM2var = [BM2var BM2Step];
end
BM2var = [BM2var ones(1,length(TimeVec)-length(BM2var))*2^32];
%BM2var = ones(1,length(TimeVec))*0.3;

TRMvar = ones(1,length(TimeVec))*0.1;

%Add all noise vectors into an noise matrix
R_dyn = [ACLvar;BM1var;BM2var;TRMvar];
R_dyn_m = diag(R_dyn(:,1)');
for n = 2:length(TimeVec)
    R_dyn_m = cat(3,R_dyn_m,diag(R_dyn(:,n)'));
end
R_dyn_t = timeseries(R_dyn,TimeVec);


%Static System noise:
HGT = 0;
SPE = 0;
ACEL = 70;
PRE = 0.1;
TMP = 0;
DTMP = 0.1;
Q = diag([HGT;SPE;ACEL;PRE;TMP;DTMP]);

%Dynamic Sytem noise:
HGT = ones(1,length(TimeVec))*0;
SPE = ones(1,length(TimeVec))*0;
ACEL = [100 100 100 50 30 ones(1,length(TimeVec)-5)*20];
PRE = ones(1,length(TimeVec))*0.1;
TMP = ones(1,length(TimeVec))*0;
DTMP = ones(1,length(TimeVec))*0.1;
Q_dyn = [HGT;SPE;ACEL;PRE;TMP;DTMP];  

%Add all noise vectors into an noise matrix
Q_dyn_m = diag(Q_dyn(:,1)');
for n = 2:length(TimeVec)
    Q_dyn_m = cat(3,Q_dyn_m,diag(Q_dyn(:,n)'));
end
Q_dyn_t = timeseries(Q_dyn,TimeVec);



%% Initialize
u = zeros(1,length(TimeVec));                       %Input vector is zero
y = [a_mes;p_mes_1;p_mes_2;T_mes];        %Output are the measurements
y_t = timeseries(y,TimeVec);
x0 = [0;0;0;Po;T(1);0];                             %Start points
x =  x0;                   %Is reality
P0 = eye(6);

%% Excecute Simulation Static
disp('Simulation start..');
sim('RocketSE');
disp('...finished!');

%% Plot
figure('Name','Real flight vs estimation simulink');
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
hold off;

%% Loop
u = zeros(1,length(TimeVec));                       %Input vector is zero
y = [a_mes;p_mes_1;p_mes_2;T_mes];                  %Output are the measurements
y_t = timeseries(y,TimeVec);
x = [0;0;0;Po;T(1);0];                              %Start Vector should be like this
P = eye(6);                                         %Standart can maybe be increased

x_est_loop = zeros(size(x,1),length(TimeVec));      %Vector for the SE values
for k = 1:length(TimeVec)
    K = P*C'*pinv(C*P*C' + R_dyn_m(:,:,k));
    x = x + K*(y(:,k) - C*x);
    P = (eye(6)-K*C)*P;
    
    x_est_loop(:,k) = x;                            %Save data from the Sensor fusion
    
    x = Ad*x + Bd*u(k);
    P = Ad*P*Ad' + Q_dyn_m(:,:,k); %Gd*Q*Gd';

end

%% Plot
figure('Name','Real flight vs estimation Self calculated');
plot(TimeVec,h);
grid on;
hold on;
plot(TimeVec,x_est_loop(1,:)); 
plot(X_estimatd.time,X_estimatd.signals.values(:,1));
plot(TimeVec,v);
plot(TimeVec,x_est_loop(2,:));
plot(TimeVec,a);
plot(TimeVec,x_est_loop(3,:));
plot(TimeVec,p);
plot(TimeVec,x_est_loop(4,:));
hold off;
legend('real Height','estiamted Height','estimated Height Simulink','real Speed','estimated Speed','real acceloration','estimated acceloration','real pressure','estimated pressure');
ylabel('height & accelaration');
xlabel('Time [s]');
