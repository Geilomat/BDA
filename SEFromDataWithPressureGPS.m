% In this script the State Estimation from given Sensor Data or self produced should be
% compared to the ground truth.

clear all; close all;


%% Initalize the System

% x1 = Height, x2 = Velocity,x3 = Acceloration, x4 = Pressure, x5 = Temp,
% x6 = Temp_dot
% Tempteratur at start = 15 C° -> 288.15 K°
load('PressLookUp.mat')

A = [0 1 0 0 0 0;0 0 1 0 0 0; 0 0 0 0 0 0;0 PressLookUp(1,1) 0 0 0 PressLookUp(2,1);0 0 0 0 0 1;0 0 0 0 0 0];
B = [0;0;0;0;0;0];                %No direct input
C = [1 0 0 0 0 0;0 0 1 0 0 0;0 0 0 1 0 0;0 0 0 1 0 0;0 0 0 0 1 0];          %Output ist Height and Acceleration
D = [0];
G = [0 0 0;0 0 0;1 0 0 ;0 1 0;0 0 0;0 0 1];                %System noise only on acceloration



% Get the time vector from the ARIS simulation.
%load('TimeFromHassan.mat');
load('TimeSimu.mat')
% Interpolate to get to 1 ms sampling time.
TimeVec = t';
Tau = TimeVec(end)/length(TimeVec);
%TauData =  
%TauDiff = round(TauData/Tau);
%TimeVec = interp(TimeVec,TauDiff);
%Tau =  round(TimeVec(end)/length(TimeVec)*1000)/1000;


Sys = ss(A,B,C,D,Tau)

 %Diskretierung der Systemmatritzen
Ad = expm(A*Tau);     
Gd = Ad * G;
Bd = Ad * B;

% Dummy vairable for Simualation
simin = zeros(1,length(TimeVec));

disp(['Size of A: ' num2str(size(A))]);

%% Observability test;
disp(['Rank: ' num2str(rank([C;C*Ad;C*Ad*Ad]))]);

%% Read sensor Data
load('StateSimu.mat')

% Get the heigth vector from the ARIS simulation.
h = state(:,3)';
%h = interp(hvec,5);

% plot it
figure('Name','Real Data')
plot(TimeVec,h,'b') %%Original bahn
legend('Real height in z');


%% Produce noisless sensor Data self

figure('Name','Real Data');
% get acceloration by differentiate height:
v = diff(h)/(Tau);
a = diff(v)/(Tau);
% Ad zeros to maintain vector length
v = [v 0];
a = [a 0 0];

% get height of GPS by deleting engouh values so it becomes 5Hz sample rate
% and then ad Zero Order Hold to get static value
GPSTau = 1;
h_GPS = zeros(1, length(TimeVec));
for k =  1:(length(h)/round(GPSTau/Tau))+1
    for t =  1:round(GPSTau/Tau)
        if (k-1)*round(GPSTau/Tau)+t <= length(TimeVec)
        h_GPS((k-1)*round(GPSTau/Tau)+t) = h((k-1)*round(GPSTau/Tau)+1);
        end
   end
end
%h_GPS = [h_GPS ones(1,20)*h_GPS(end)];

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
for k =  1:(length(p)/round(P1Tau/Tau))+1
    for t =  1:round(P1Tau/Tau)
        if (k-1)*round(P1Tau/Tau)+t <= length(TimeVec)
        P1((k-1)*round(P1Tau/Tau)+t) = p((k-1)*round(P1Tau/Tau)+1);
        end
   end
end
                            
%P2 Has more noise but samples faster
P2Tau = 1/100;
P2 = zeros(1,length(TimeVec));
for k =  1:(length(p)/round(P2Tau/Tau))+1
    for t =  1:round(P2Tau/Tau)
        if (k-1)*round(P2Tau/Tau)+t <= length(TimeVec)
        P2((k-1)*round(P2Tau/Tau)+t) = p((k-1)*round(P2Tau/Tau)+1);
        end
   end
end                           

plot(TimeVec,h);
hold on;
plot(TimeVec,h_GPS)
plot(TimeVec,a);
plot(TimeVec,T)
plot(TimeVec,p);
plot(TimeVec,P1);
plot(TimeVec,P2);
legend('Real height in z','GPS heigt in z','Real Acceloration','Assumed Temperature in Kelvin','Real Pressure','Assumed Pressure 1','Assumed Pressure 2');
hold off;

%% Add noise to sensor data

T_var_brn = 8.4040e-04;
p_var_brn = 1.7034;
p2_var_brn = p_var_brn * 2;
p_var_upflight = 0.7034;
p2_var_upflight = p_var_upflight * 2;
a_var_brn = 0.0128;
a_var_upflight = 0.0028;
aofst_var_brn = 0.0001;
GPS_var = 0.1;

% with EPFL data
%load('sensorNoiseTir2.mat')
%load('sensorNoiseTir1.mat')

% Temperatur
T_mes = T + randn(1,length(T)).*sqrt(T_var_brn);

% Acceleloration > Add a offset and more noise if while the motor is
% burning
a_mes = zeros(1,length(a));
a_offset = 4;
for k = 1:length(a)
    if a(k) > 20
    a_mes(k) = a(k) + randn * sqrt(a_var_brn) + (a_offset + randn*aofst_var_brn);
    else
    a_mes(k) = a(k) + randn * sqrt(a_var_upflight) + (a_offset + randn*aofst_var_brn);    
    end
end

% Pressure -> more noise if the motor is burning
for k = 1:length(a)
    if a(k) > 20
    p_mes_1 = p + randn(1,length(P1)).*sqrt(p_var_brn);
    p_mes_2 = p + randn(1,length(P2)).*sqrt(p2_var_brn);
    else
    p_mes_1 = p + randn(1,length(P1)).*sqrt(p_var_upflight);
    p_mes_2 = p + randn(1,length(P2)).*sqrt(p2_var_upflight);    
    end
end
% GPS
h_mes_GPS = h_GPS + randn(1,length(h_GPS)).*sqrt(GPS_var);

% T_mes = awgn(T,40,'measured');
% h_mes_GPS = awgn(h_GPS,80,'measured');
% p_mes_1 = awgn(p,45,'measured');
% p_mes_2 = awgn(p,40,'measured');
% a_mes = awgn(a,30,'measured');

figure('Name','Noise Data');
hold on;
plot(h_mes_GPS);
plot(h_GPS);
plot(p_mes_1);
plot(p_mes_2);
plot(p)
plot(a_mes);
plot(a);
plot(T_mes);
plot(T);
legend('GPSnoise','GPS','Pressure 1 noise','Pressure 2 noise','Pressure','Acceloration Measured','Acceleration','Temperatur noise','Temperatur');
hold off;

%% Create Noise Matrices

% Static Sensor noise
% Calculate the optimal variance
GPSvar = (1/(length(h_GPS)-1)*sum((h_mes_GPS-h_GPS).^2));
ACLvar = (1/(length(a)-1)*sum((a_mes-a).^2));
BM1var = (1/(length(P1)-1)*sum((p_mes_1-P1).^2));
BM2var = (1/(length(P2)-1)*sum((p_mes_2-P2).^2));
TRMvar = (1/(length(T)-1)*sum((T_mes-T).^2));
R = diag([GPSvar ACLvar BM1var BM2var TRMvar]);
ZV = zeros(1,length(TimeVec));

%Dynamic Sensor noise:
GPSStep = [GPSvar ones(1,round(GPSTau/Tau) - 1)*2^32];
GPSvar = [];
for n = 1:round(length(TimeVec)/length(GPSStep))-1
    GPSvar = [GPSvar GPSStep];
end
GPSvar = [GPSvar ones(1,length(TimeVec)-length(GPSvar))*2^32];

ACLvar = ones(1,length(TimeVec))*ACLvar;

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

TRMvar = ones(1,length(TimeVec))*TRMvar;

%Add all noise vectors into an noise matrix
R_dyn = [GPSvar;ACLvar;BM1var;BM2var;TRMvar];
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
PRE = [zeros(1,10) ones(1,length(TimeVec)-20)*0.7 zeros(1,10)];         %Due to linearization it gets less accurate in the middle
TMP = ones(1,length(TimeVec))*0;
DTMP = ones(1,length(TimeVec))*0.1;
%Q_dyn = [HGT;SPE;ACEL;PRE;TMP;DTMP]; 
Q_dyn = [ACEL;PRE;DTMP];

%Add all noise vectors into an noise matrix
Q_dyn_m = diag(Q_dyn(:,1)');
for n = 2:length(TimeVec)
    Q_dyn_m = cat(3,Q_dyn_m,diag(Q_dyn(:,n)'));
end
Q_dyn_t = timeseries(Q_dyn,TimeVec);

%% Initialize
u = zeros(1,length(TimeVec));                       %Input vector is zero
y = [h_mes_GPS;a_mes;p_mes_1;p_mes_2;T_mes];        %Output are the measurements
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
y = [h_mes_GPS;a_mes;p_mes_1;p_mes_2;T_mes];        %Output are the measurements
y_t = timeseries(y,TimeVec);
x = [0;0;0;Po;T(1);0];                              %Start Vector should be like this
P = eye(6);                                         %Standart can maybe be increased

x_est_loop = zeros(size(x,1),length(TimeVec));      %Vector for the SE values
disp('Loop start..');
for k = 1:length(TimeVec)
    K = P*C'*pinv(C*P*C' + R_dyn_m(:,:,k));
    x = x + K*(y(:,k) - C*x);
    P = (eye(6)-K*C)*P;
    
    x_est_loop(:,k) = x;                            %Save data from the Sensor fusion
    
    x = Ad*x + Bd*u(k);
    P = Ad*P*Ad' + Gd* Q_dyn_m(:,:,k)*Gd'; %Gd*Q*Gd';

end
disp('...finished!');

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
ylabel('height & speed & accelaration & pressure');
xlabel('Time [s]');



%% Loop 2
% Initalize the System

% x1 = Height, x2 = Velocity,x3 = Acceloration,x4 = aoffset x5 = Pressure
% x6 = Temp, x7 = Temp_dot
% Tempteratur at start = 15 C° -> 288.15 K°
load('PressLookUp.mat');

A = [0 1 0 0 0 0 0;
    0 0 1 0 0 0 0;
    0 0 0 0 0 0 0;
    0 0 0 0 0 0 0;
    0 PressLookUp(1,1) 0 0 0 0 PressLookUp(2,1);
    0 0 0 0 0 0 1;
    0 0 0 0 0 0 0];
B = [0;0;0;0;0;0;0];                %No direct input
C = [1 0 0 0 0 0 0;
    0 0 1 -1 0 0 0;
    0 0 0 0 1 0 0;
    0 0 0 0 1 0 0;
    0 0 0 0 0 1 0];          %Output ist Height, Pressures and Temperature
D = [0];
G = [0 0 0 0;
    0 0 0 0;
    1 0 0 0;
    0 1 0 0;
    0 0 1 0;
    0 0 0 0;
    0 0 0 1];                %System noise on acceloration, acc offset, pressure an Tdot

Sys = ss(A,B,C,D,Tau)

 %Diskretierung der Systemmatritzen
Ad = expm(A*Tau);     
Gd = Ad * G;
Bd = Ad * B;

% Dummy vairable for Simualation
simin = zeros(1,length(TimeVec));

disp(['Size of A: ' num2str(size(A))]);

%% Observability test;
disp(['Rank: ' num2str(rank([C;C*Ad;C*Ad*Ad]))]);

%% Add Offset to system noise
%Static System noise:
HGT = 0;
SPE = 0;
ACEL = 70;
ACELOFS = 0.01;
PRE = 0.1;
TMP = 0;
DTMP = 0.1;
Q = diag([ACEL;ACELOFS;PRE;TMP;DTMP]);

%Dynamic Sytem noise:
HGT = ones(1,length(TimeVec))*0;
SPE = ones(1,length(TimeVec))*0;
ACEL = [100 100 100 50 30 ones(1,length(TimeVec)-5)*10];
ACELOFS = ones(1,length(TimeVec))*ACELOFS;
PRE = [zeros(1,10) ones(1,length(TimeVec)-20)*0.7 zeros(1,10)];         %Due to linearization it gets less accurate in the middle
TMP = ones(1,length(TimeVec))*0;
DTMP = ones(1,length(TimeVec))*0.1;
%Q_dyn = [HGT;SPE;ACEL;PRE;TMP;DTMP]; 
Q_dyn = [ACEL;ACELOFS;PRE;DTMP];

%Add all noise vectors into an noise matrix
Q_dyn_m = diag(Q_dyn(:,1)');
for n = 2:length(TimeVec)
    Q_dyn_m = cat(3,Q_dyn_m,diag(Q_dyn(:,n)'));
end

%% Do estimation Loop

u = zeros(1,length(TimeVec));                       %Input vector is zero
y = [h_mes_GPS;a_mes;p_mes_1;p_mes_2;T_mes];        %Output are the measurements
y_t = timeseries(y,TimeVec);
x = [0;0;0;0;Po;T(1);0];                            %Start Vector should be like this
P = eye(7);                                         %Standart can maybe be increased

x_est_loop2 = zeros(size(x,1),length(TimeVec));      %Vector for the SE values
disp('Loop 2 start..');
for k = 1:length(TimeVec)
    K = P*C'*pinv(C*P*C' + R_dyn_m(:,:,k));
    x = x + K*(y(:,k) - C*x);
    P = (eye(7)-K*C)*P;
    
    x_est_loop2(:,k) = x;                            %Save data from the Sensor fusion
    
    x = Ad*x + Bd*u(k);
    P = Ad*P*Ad' + Gd* Q_dyn_m(:,:,k)*Gd'; %Gd*Q*Gd';

end
disp('...finished!');

%% Plot Loop 2

figure('Name','Real flight 2 vs estimation Self calculated');
plot(TimeVec,h);
grid on;
hold on;
plot(TimeVec,x_est_loop2(1,:)); 
plot(TimeVec,x_est_loop(1,:));
plot(TimeVec,v);
plot(TimeVec,x_est_loop2(2,:));
plot(TimeVec,a);
plot(TimeVec,x_est_loop2(3,:));
plot(TimeVec,x_est_loop2(4,:));
plot(TimeVec,p);
plot(TimeVec,x_est_loop2(5,:));
hold off;
legend('real Height','estiamted Height','estimated Height Simulink','real Speed','estimated Speed','real acceloration','estimated acceloration','acceleration offset','real pressure','estimated pressure');
ylabel('height & speed & accelaration & pressure');
xlabel('Time [s]');


%% Loop 3
A = [0 1 0 0;0 0 1 0; 0 0 0 0;-0.00649 0 0 0];
B = [0;0;0;0];                %No direct input
C = [1 0 0 0;1 0 0 0;1 0 0 0;0 0 1 0;0 0 0 1];          %Output ist Height and Acceleration
D = [0];
G = [0;0;1;0];                %System noise only on acceloration


 %Diskretierung der Systemmatritzen
Ad = expm(A*Tau);     
Gd = Ad * G;
Bd = Ad * B;

Q_dyn = [HGT;SPE;ACEL;TMP];  

%Add all noise vectors into an noise matrix
Q_dyn_m = diag(Q_dyn(:,1)');
for n = 2:length(TimeVec)
    Q_dyn_m = cat(3,Q_dyn_m,diag(Q_dyn(:,n)'));
end

u = zeros(1,length(TimeVec));                       %Input vector is zero
y = [h_mes_GPS;a_mes;p_mes_1;p_mes_2;T_mes];        %Output are the measurements
y_t = timeseries(y,TimeVec);
x = [0;0;0;T(1)];                              %Start Vector should be like this
P = eye(4);                                         %Standart can maybe be increased
Height1 = 0;
Height2 = 0;
Temp = T(1);


x_est_loop3 = zeros(size(x,1),length(TimeVec));      %Vector for the SE values
disp('Loop 3 start..');
for k = 1:length(TimeVec)
    K = P*C'*pinv(C*P*C' + R_dyn_m(:,:,k));
    Height1 = CalcHeight(Po,p_mes_1(k),T_mes(k),0,true);
    Height2 = CalcHeight(Po,p_mes_2(k),T_mes(k),0,true);
    Temp = T_mes(k);
    x = x + K*([h_mes_GPS(k);Height1;Height2;a_mes(k);Temp] - C*x);
    P = (eye(4)-K*C)*P;
    
    x_est_loop3(:,k) = x;                            %Save data from the Sensor fusion
    
    x = Ad*x + Bd*u(k);
    P = Ad*P*Ad' + Q_dyn_m(:,:,k); %Gd*Q*Gd';

end
disp('...finished!');

%% Plot

figure('Name','Real flight 3 vs estimation Self calculated');
plot(TimeVec,h);
grid on;
hold on;
plot(TimeVec,x_est_loop3(1,:)); 
plot(TimeVec,x_est_loop(1,:));
plot(TimeVec,v);
plot(TimeVec,x_est_loop3(2,:));
plot(TimeVec,a);
plot(TimeVec,x_est_loop3(3,:));
plot(TimeVec,p);
plot(TimeVec,x_est_loop3(4,:));
hold off;
legend('real Height','estiamted Height','estimated Height Simulink','real Speed','estimated Speed','real acceloration','estimated acceloration','real pressure','estimated pressure');
ylabel('height & speed & accelaration & pressure');
xlabel('Time [s]');

%% Difference between estimation and ground truth
diff = abs(h-x_est_loop(1,:));
display(['Loop 1 "Normal" Max difference:' num2str(max(diff)) ' min difference:' num2str(min(diff)) ' average difference:' num2str(sum(diff)/length(diff))]);
diff = abs(h-x_est_loop2(1,:));
display(['Loop 2 Acceloration with offset Max difference:' num2str(max(diff)) ' min difference:' num2str(min(diff)) ' average difference:' num2str(sum(diff)/length(diff))]);
diff = abs(h-x_est_loop3(1,:));
display(['Loop 3 Pressure as height Max difference:' num2str(max(diff)) ' min difference:' num2str(min(diff)) ' average difference:' num2str(sum(diff)/length(diff))]);


% %%
% figure('Name','Generated sensor data');
% hold on;
% grid on;
% plot(TimeVec,h);
% plot(TimeVec,h_GPS)
% plot(TimeVec,a);
% plot(TimeVec,T)
% plot(TimeVec,p);
% legend('Real height in z','GPS heigt in z','Acceloration','Temperature','Pressure');
% hold off;
% ylabel('Height [m], Accelaration [m/s^2], Temperature [K°], Pressure [hPa]');
% xlabel('Time [s]');
