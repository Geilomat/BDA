% In this script the State Estimation from given Sensor Data or self produced should be
% compared to the ground truth.

clear all; close all;


%% Initalize the System
disp('System 1 with x1 = Height,x2 = Velocity,x3 = Acceloration,x4 = Pressure,x5 = Temp,6 = Temp_dot');



% x1 = Height, x2 = Velocity,x3 = Acceloration, x4 = Pressure, x5 = Temp,
% x6 = Temp_dot
% Tempteratur at start = 15 C° -> 288.15 K°
load('PressLookUp.mat')

A = [0 1 0 0 0 0;
    0 0 1 0 0 0; 
    0 0 0 0 0 0;
    0 PressLookUp(2,1) 0 0 0 PressLookUp(1,1);
    0 0 0 0 0 1;
    0 0 0 0 0 0];
B = [0;0;0;0;0;0];                %No direct input
C = [1 0 0 0 0 0;
    0 0 1 0 0 0;
    0 0 0 1 0 0;
    0 0 0 1 0 0;
    0 0 0 0 1 0];          %Output ist Height and Acceleration
D = [0];
G = [0 0 0;0 0 0;1 0 0 ;0 1 0;0 0 0;0 0 1];                %System noise only on acceloration

% Get the time vector from the ARIS simulation.
%load('TimeFromHassan.mat');
load('RoroState.mat')
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

disp(['Size of A: ' num2str(size(A))]);

%% Observability test;
disp(['Rank: ' num2str(rank([C;C*Ad;C*Ad*Ad]))]);

%% Read sensor Data
%load('StateSimu.mat')

% Get the heigth vector from the ARIS simulation.
h = state(:,3)';

% plot it
figure('Name','Real Data')
plot(TimeVec,h,'b')             %Original trajectory
legend('Real height in z');


%% Produce noisless sensor Data self

figure('Name','Real Data');
% get acceloration by differentiate height:
v = diff(h)/(Tau);
%a = diff(v)/(Tau);
% Ad zeros to maintain vector length
v = [v v(end)];
%a = [a a(end) a(end)];
anew = resample(imu_a(:,3),imu_t,1000);
a = anew(1:length(h))';

% Find burnout index
k = 20;
while a(k) > 20
    k = k+1;
end
T_brn_ind = k;


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
p = Po*(1-(0.0065*h)./T0).^5.255;

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

% Temperature measurments of the first barometer
T1 = zeros(1,length(TimeVec));
for k =  1:(length(T)/round(P1Tau/Tau))+1
    for t =  1:round(P1Tau/Tau)
        if (k-1)*round(P1Tau/Tau)+t <= length(TimeVec)
        T1((k-1)*round(P1Tau/Tau)+t) = T((k-1)*round(P1Tau/Tau)+1);
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

% Temperature measurements of the second barometer
T2 = zeros(1,length(TimeVec));
for k =  1:(length(p)/round(P2Tau/Tau))+1
    for t =  1:round(P2Tau/Tau)
        if (k-1)*round(P2Tau/Tau)+t <= length(TimeVec)
        T2((k-1)*round(P2Tau/Tau)+t) = T((k-1)*round(P2Tau/Tau)+1);
        end
   end
end        

phi = zeros(1,length(TimeVec));
% Integrate over more or less random values to get a pitch angle
% representet with air current etc
for k = 2:length(TimeVec)  
    if k > T_brn_ind
        phi(k) = phi(k-1) + randn(1,1) * 1000 * Tau;
    else
       phi(k) = phi(k-1) + randn(1,1) * 20 * Tau; 
    end 
end
% Filter the noise phi angle with a moving average filter
n = 10000;
phi = filter((1/n)*ones(1,n),1,phi);

plot(TimeVec,h);
hold on;
plot(TimeVec,h_GPS)
plot(TimeVec,a);
plot(TimeVec,T)
plot(TimeVec,p);
plot(TimeVec,P1);
plot(TimeVec,P2);
plot(TimeVec,phi);
legend('Real height in z','GPS heigt in z','Real Acceloration','Assumed Temperature in Kelvin','Real Pressure','Assumed Pressure 1','Assumed Pressure 2','Pitch angle');
hold off;

%% Add noise to sensor data
% Load the AR models of the different noises
load('h_a.mat');

% T_var_brn = 8.4040e-04;
% p_var_brn = 1.7034;
%p2_var_brn = p_var_brn * 2;
% p_var_upflight = 0.7034;
%p2_var_upflight = p_var_upflight * 2;
% a_var_brn = 0.0128;
% a_var_upflight = 0.0028;
aofst_var_brn = 0.0001;
aofst_var_upflight = 0.0001;
GPS_var = 0.1;

% with EPFL data
%load('sensorNoiseTir2.mat')
%load('sensorNoiseTir1.mat')


% Acceleloration > Add a offset and more noise if while the motor is
% burning
a_mes = zeros(1,length(a));
a_offset = 4;
% Generate noiese Vecotrs:
a_noise_brn = filter(1,h_brn,randn(1,T_brn_ind) * sqrt(varBrn));
aofst_noise_brn = filter(1,h_brn,randn(1,T_brn_ind) * sqrt(aofst_var_brn));
a_noise_upflight = filter(1,h_upflight,randn(1,length(a)-T_brn_ind) * sqrt(varUpflight));
aofst_noise_upflight = filter(1,h_upflight,randn(1,length(a)-T_brn_ind) * sqrt(aofst_var_upflight));

for k = 1:length(a)
     if k <= T_brn_ind
     a_mes(k) = a(k) + a_noise_brn(k) + a_offset + aofst_noise_brn(k);
     else
     a_mes(k) = a(k) + a_noise_upflight(k-T_brn_ind) + a_offset + aofst_noise_upflight(k-T_brn_ind);    
     end
end

% Pitch angle, using the same noise as acceloration cause it should have
% more or less the sam properties -> Both IMU 
% Maybe change this in a later implementation
phi_noise_brn = filter(1,h_brn,randn(1,T_brn_ind) * sqrt(varBrn));
phi_noise_upflight = filter(1,h_upflight,randn(1,length(a)-T_brn_ind) * sqrt(varUpflight));
for k = 1:length(a)
    if k <= T_brn_ind
        phi_mes(k) = phi(k) + phi_noise_brn(k);
    else
        phi_mes(k) = phi(k) + phi_noise_upflight(k-T_brn_ind);
    end
end

load('h_p.mat');
p2_var_brn = varBrn * 2;
p2_var_upflight = varUpflight * 2;
% Pressure -> more noise if the motor is burning
p_noise_brn = filter(1,h_brn,randn(1,T_brn_ind) * sqrt(varBrn));
p2_noise_brn = filter(1,h_brn,randn(1,T_brn_ind) * sqrt(p2_var_brn));
p_noise_upflight = filter(1,h_upflight,randn(1,length(a)-T_brn_ind) * sqrt(varUpflight));
p2_noise_upflight = filter(1,h_upflight,randn(1,length(a)-T_brn_ind) * sqrt(p2_var_upflight));

for k = 1:length(a)
    if k <= T_brn_ind
    p_mes_1(k) = p(k) + p_noise_brn(k);
    p_mes_2(k) = p(k) + p2_noise_brn(k);
    else
    p_mes_1(k) = p(k) + p_noise_upflight(k-T_brn_ind);
    p_mes_2(k) = p(k) + p2_noise_upflight(k-T_brn_ind);
    end
end

% GPS
load('h_GPS.mat');

h_mes_GPS = h_GPS + filter(1,h_upflight,randn(1,length(h_GPS)).*sqrt(varUpflight));


% Temperatur
load('h_T.mat');
T1_noise_brn = filter(1,h_brn,randn(1,T_brn_ind) * sqrt(varBrn));
T1_noise_upflight = filter(1,h_upflight,randn(1,length(a)-T_brn_ind) * sqrt(varUpflight));
T2_noise_brn = filter(1,h_brn,randn(1,T_brn_ind) * sqrt(varBrn))*2;
T2_noise_upflight = filter(1,h_upflight,randn(1,length(a)-T_brn_ind) * sqrt(varUpflight))*2;

for k = 1:length(a)
    if k <= T_brn_ind
    T1_mes(k) = T1(k) + T1_noise_brn(k);
    T2_mes(k) = T2(k) + T2_noise_brn(k);
    else
    T1_mes(k) = T1(k) + T1_noise_upflight(k-T_brn_ind);
    T2_mes(k) = T2(k) + T1_noise_upflight(k-T_brn_ind);
    end
end


%load('h_phi.mat')
%T_mes = T + randn(1,length(T)).*sqrt(T_var_brn);

% T_mes = awgn(T,40,'measured');
% h_mes_GPS = awgn(h_GPS,80,'measured');
% p_mes_1 = awgn(p,45,'measured');
% p_mes_2 = awgn(p,40,'measured');
% a_mes = awgn(a,30,'measured');

% T_mes = T;
% h_mes_GPS = h_GPS;
% p_mes_1 = P1;
% p_mes_2 = P2;
% a_mes = a;

figure('Name','Noise Data');
hold on;
plot(h_mes_GPS);
plot(h_GPS);
plot(p_mes_1);
plot(p_mes_2);
plot(p)
plot(a_mes);
plot(a);
plot(T1_mes);
plot(T);
plot(phi_mes);
plot(phi);
legend('GPSnoise','GPS','Pressure 1 noise','Pressure 2 noise','Pressure','Acceloration Measured','Acceleration','Temperatur noise','Temperatur','Pitch angle noise','Pitch angle');
hold off;

%% Create Noise Matrices

% Static Sensor noise
% Calculate the optimal variance
GPSvar = (1/(length(h_GPS)-1)*sum((h_mes_GPS-h_GPS).^2));
ACLvar = (1/(length(a)-1)*sum((a_mes-a).^2));
BM1var = (1/(length(P1)-1)*sum((p_mes_1-P1).^2));
BM2var = (1/(length(P2)-1)*sum((p_mes_2-P2).^2));
TRMvar = (1/(length(T)-1)*sum((T1_mes-T).^2));
PHIvar = (1/(length(phi)-1)*sum((phi_mes-phi).^2));
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
y = [h_mes_GPS;a_mes;p_mes_1;p_mes_2;T1_mes];        %Output are the measurements
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
y = [h_mes_GPS;a_mes;p_mes_1;p_mes_2;T1_mes];        %Output are the measurements
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
%plot(X_estimatd.time,X_estimatd.signals.values(:,1));
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
disp('System 2 with x1 = Height,x2 = Velocity,x3 = Acceloration,x4 = aoffset x5 = Pressure,x6 = Temp,x7 = Temp_dot')

% x1 = Height, x2 = Velocity,x3 = Acceloration,x4 = aoffset x5 = Pressure
% x6 = Temp, x7 = Temp_dot
% Tempteratur at start = 15 C° -> 288.15 K°
load('PressLookUp.mat');

A = [0 1 0 0 ;%0 0 0;
    0 0 1 0 ;%0 0 0;
    0 0 0 0 ;%0 0 0;
    0 0 0 0 ];%0 0 0;
    %0 PressLookUp(2,1) 0 0 0 0 PressLookUp(1,1);
    %0 0 0 0 0 0 1;
    %0 0 0 0 0 0 0];
B = [0;0;0;0];%;0;0;0];                %No direct input
C = [1 0 0 0;% 0 0 0;
    0 0 1 1 ];%0 0 0;
    %0 0 0 0 ;%1 0 0;
    %0 0 0 0 ];%1 0 0;
    %0 0 0 0 ]%0 1 0];          %Output ist Height, Pressures and Temperature
D = [0];
G = [0 0;% 0 0;
    0 0;% 0 0;
    1 0;% 0 0;
    0 1]% 0 0;
    %0 0 1 0;
    %0 0 0 0;
    %0 0 0 1];                %System noise on acceloration, acc offset, pressure an Tdot

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
ACELOFS = 0.0001;
PRE = 0.1;
TMP = 0;
DTMP = 0.1;
Q = diag([ACEL;ACELOFS;PRE;TMP;DTMP]);

%Dynamic Sytem noise:
HGT = ones(1,length(TimeVec))*0;
SPE = ones(1,length(TimeVec))*0;
ACEL = [100 100 100 50 30 ones(1,length(TimeVec)-5)*.1];
ACELOFS = ones(1,length(TimeVec))*ACELOFS;
PRE = [zeros(1,10) ones(1,length(TimeVec)-20)*0.7 zeros(1,10)];         %Due to linearization it gets less accurate in the middle
TMP = ones(1,length(TimeVec))*0;
DTMP = ones(1,length(TimeVec))*0.1;
%Q_dyn = [HGT;SPE;ACEL;PRE;TMP;DTMP]; 
Q_dyn = [ACEL;ACELOFS];%;PRE;DTMP];

%Add all noise vectors into an noise matrix
Q_dyn_m = diag(Q_dyn(:,1)');
for n = 2:length(TimeVec)
    Q_dyn_m = cat(3,Q_dyn_m,diag(Q_dyn(:,n)'));
end

%Add all noise vectors into an noise matrix
R_dyn = [GPSvar;ACLvar];%;BM1var;BM2var;TRMvar];
R_dyn_m = diag(R_dyn(:,1)');
for n = 2:length(TimeVec)
    R_dyn_m = cat(3,R_dyn_m,diag(R_dyn(:,n)'));
end
R_dyn_t = timeseries(R_dyn,TimeVec);

%% Do estimation Loop

u = zeros(1,length(TimeVec));                       %Input vector is zero
y = [h_mes_GPS;a_mes];%;p_mes_1;p_mes_2;T_mes];        %Output are the measurements
y_t = timeseries(y,TimeVec);
x = [0;0;0;0]%;Po;T(1);0];                            %Start Vector should be like this
P = eye(4);                                         %Standart can maybe be increased

x_est_loop2 = zeros(size(x,1),length(TimeVec));      %Vector for the SE values
disp('Loop 2 start..');
for k = 1:length(TimeVec)
    K = P*C'*pinv(C*P*C' + R_dyn_m(:,:,k));
    x = x + K*(y(:,k) - C*x);
    P = (eye(4)-K*C)*P;
    
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
% plot(TimeVec,p);
% plot(TimeVec,x_est_loop2(5,:));
hold off;
legend('real Height','estiamted Height','estimated Height Simulink','real Speed','estimated Speed','real acceloration','estimated acceloration','acceleration offset','real pressure','estimated pressure');
ylabel('height & speed & accelaration & pressure');
xlabel('Time [s]');


%% Loop 3
disp('System 3 with x1=Height,x2=Speed,x3=Acceleration,x4=Acceleration Offset,x5=Temperature');
% Initialize the system

A = [0 1 0 0 0;
    0 0 1 0 0;
    0 0 0 0 0;
    0 0 0 0 0;
    -0.000649 0 0 0 0];
B = [0;0;0;0;0];                %No direct input
C = [1 0 0 0 0;
    0 0 1 -1 0;
    1 0 0 0 0;
    1 0 0 0 0;
    0 0 0 0 1];          
D = [0];
G = [0 0;
    0 0;
    1 0;
    0 1;
    0 0];                %System noise only on acceloration


 %Diskretierung der Systemmatritzen
Ad = expm(A*Tau);     
Gd = Ad * G;
Bd = Ad * B;

disp(['Size of A: ' num2str(size(A))]);

%% Observability test;
disp(['Rank: ' num2str(rank([C;C*Ad;C*Ad*Ad]))]);

%% System noise implementation

%Q_dyn = [HGT;SPE;ACEL;TMP];  
Q_dyn = [ACEL;ACELOFS];
%Add all noise vectors into an noise matrix
Q_dyn_m = diag(Q_dyn(:,1)');
for n = 2:length(TimeVec)
    Q_dyn_m = cat(3,Q_dyn_m,diag(Q_dyn(:,n)'));
end

%Add all noise vectors into an noise matrix
R_dyn = [GPSvar;ACLvar;BM1var;BM2var;TRMvar];
R_dyn_m = diag(R_dyn(:,1)');
for n = 2:length(TimeVec)
    R_dyn_m = cat(3,R_dyn_m,diag(R_dyn(:,n)'));
end
R_dyn_t = timeseries(R_dyn,TimeVec);


%% Loop initialization and loop

u = zeros(1,length(TimeVec));                       %Input vector is zero
y = [h_mes_GPS;a_mes;p_mes_1;p_mes_2;T1_mes];        %Output are the measurements
y_t = timeseries(y,TimeVec);
x = [0;0;0;0;T(1)];                                   %Start Vector should be like this
P = eye(5);                                         %Standart can maybe be increased
Height1 = 0;
Height2 = 0;
Temp = T(1);


x_est_loop3 = zeros(size(x,1),length(TimeVec));      %Vector for the SE values
disp('Loop 3 start..');
for k = 1:length(TimeVec)
    K = P*C'*pinv(C*P*C' + R_dyn_m(:,:,k));
    Height1 = CalcHeight(Po,p_mes_1(k),T0,0,true);
    Height2 = CalcHeight(Po,p_mes_2(k),T0,0,true);
    Temp = T1_mes(k);
    x = x + K*([h_mes_GPS(k);a_mes(k);Height1;Height2;Temp] - C*x);
    P = (eye(5)-K*C)*P;
    
    x_est_loop3(:,k) = x;                            %Save data from the Sensor fusion
    
    x = Ad*x + Bd*u(k);
    P = Ad*P*Ad' + Gd*Q_dyn_m(:,:,k)*Gd'; %Gd*Q*Gd';

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
plot(TimeVec,x_est_loop3(4,:));
plot(TimeVec,T);
plot(TimeVec,x_est_loop3(5,:));
hold off;
legend('real Height','estiamted Height','estimated Height Simulink','real Speed','estimated Speed','real acceloration','estimated acceloration','acceloration offset','real temperature','estimated temperature');
ylabel('height & speed & accelaration & pressure');
xlabel('Time [s]');

%% Loop 4 Without Temperature

disp('System 4 with x1=Height,x2=Speed,x3=Acceleration,x4=Acceleration Offset');
% Initialize the system

A = [0 1 0 0 ;
    0 0 1 0 ;
    0 0 0 0 ;
    0 0 0 0] ;
B = [0;0;0;0];                %No direct input
C = [1 0 0 0;
    0 0 1 -1;
    1 0 0 0;
    1 0 0 0];         
D = [0];
G = [0 0;
    0 0;
    1 0;
    0 1];


 %Diskretierung der Systemmatritzen
Ad = expm(A*Tau);     
Gd = Ad * G;
Bd = Ad * B;

disp(['Size of A: ' num2str(size(A))]);

%% Observability test;
disp(['Rank: ' num2str(rank([C;C*Ad;C*Ad*Ad]))]);

%% System noise implementation

Q_dyn = [ACEL;ACELOFS];
%Add all noise vectors into an noise matrix
Q_dyn_m = diag(Q_dyn(:,1)');
for n = 2:length(TimeVec)
    Q_dyn_m = cat(3,Q_dyn_m,diag(Q_dyn(:,n)'));
end

%Add all noise vectors into an noise matrix
R_dyn = [GPSvar;ACLvar;BM1var;BM2var];
R_dyn_m = diag(R_dyn(:,1)');
for n = 2:length(TimeVec)
    R_dyn_m = cat(3,R_dyn_m,diag(R_dyn(:,n)'));
end
R_dyn_t = timeseries(R_dyn,TimeVec);


%% Loop initialization and loop

u = zeros(1,length(TimeVec));                       %Input vector is zero
y = [h_mes_GPS;a_mes;p_mes_1;p_mes_2];        %Output are the measurements
y_t = timeseries(y,TimeVec);
x = [0;0;0;0];                                   %Start Vector should be like this
P = eye(4);                                         %Standart can maybe be increased
Height1 = 0;
Height2 = 0;
Temp = T(1);


x_est_loop4 = zeros(size(x,1),length(TimeVec));      %Vector for the SE values
disp('Loop 4 start..');
for k = 1:length(TimeVec)
    K = P*C'*pinv(C*P*C' + R_dyn_m(:,:,k));
    Height1 = CalcHeight(Po,p_mes_1(k),T0,0,true);
    Height2 = CalcHeight(Po,p_mes_2(k),T0,0,true);
    Temp = T1_mes(k);
    x = x + K*([h_mes_GPS(k);a_mes(k);Height1;Height2]- C*x);
    P = (eye(4)-K*C)*P;
    
    x_est_loop4(:,k) = x;                            %Save data from the Sensor fusion
    
    x = Ad*x + Bd*u(k);
    P = Ad*P*Ad' + Gd*Q_dyn_m(:,:,k)*Gd'; %Gd*Q*Gd';

end
disp('...finished!');

%% Plot

figure('Name','Real flight vs estimation Self calculated System 4');
plot(TimeVec,h);
grid on;
hold on;
plot(TimeVec,x_est_loop4(1,:)); 
plot(TimeVec,x_est_loop(1,:));
plot(TimeVec,v);
plot(TimeVec,x_est_loop4(2,:));
plot(TimeVec,a);
plot(TimeVec,x_est_loop4(3,:));
plot(TimeVec,x_est_loop4(4,:));
hold off;
legend('real Height','estiamted Height','estimated Height Simulink','real Speed','estimated Speed','real acceloration','estimated acceloration','acceloration offset');
ylabel('height & speed & accelaration & pressure');
xlabel('Time [s]');


%% Loop 5 Without Temperature and GPS

disp('System 5 with x1=Height,x2=Speed,x3=Acceleration,x4=Acceleration Offset and without GPS');
% Initialize the system

A = [0 1 0 0 ;
    0 0 1 0 ;
    0 0 0 0 ;
    0 0 0 0] ;
B = [0;0;0;0];                %No direct input
C = [0 0 1 -1;
    1 0 0 0;
    1 0 0 0];         
D = [0];
G = [0 0;
    0 0;
    1 0;
    0 1];


 %Diskretierung der Systemmatritzen
Ad = expm(A*Tau);     
Gd = Ad * G;
Bd = Ad * B;

disp(['Size of A: ' num2str(size(A))]);

%% Observability test;
disp(['Rank: ' num2str(rank([C;C*Ad;C*Ad*Ad]))]);

%% System noise implementation

Q_dyn = [ACEL;ACELOFS];
%Add all noise vectors into an noise matrix
Q_dyn_m = diag(Q_dyn(:,1)');
for n = 2:length(TimeVec)
    Q_dyn_m = cat(3,Q_dyn_m,diag(Q_dyn(:,n)'));
end

%Add all noise vectors into an noise matrix
R_dyn = [ACLvar;BM1var;BM2var];
R_dyn_m = diag(R_dyn(:,1)');
for n = 2:length(TimeVec)
    R_dyn_m = cat(3,R_dyn_m,diag(R_dyn(:,n)'));
end
R_dyn_t = timeseries(R_dyn,TimeVec);


%% Loop initialization and loop

u = zeros(1,length(TimeVec));                       %Input vector is zero
%y_t = timeseries(y,TimeVec);
x = [0;0;0;0];                                   %Start Vector should be like this
P = eye(4);                                         %Standart can maybe be increased
Height1 = 0;
Height2 = 0;
Temp = T(1);


x_est_loop5 = zeros(size(x,1),length(TimeVec));      %Vector for the SE values
disp('Loop 5 start..');
for k = 1:length(TimeVec)
    K = P*C'*pinv(C*P*C' + R_dyn_m(:,:,k));
    Height1 = CalcHeight(Po,p_mes_1(k),T0,0,true);
    Height2 = CalcHeight(Po,p_mes_2(k),T0,0,true);
    Temp = T1_mes(k);
    x = x + K*([a_mes(k);Height1;Height2]- C*x);
    P = (eye(4)-K*C)*P;
    
    x_est_loop5(:,k) = x;                            %Save data from the Sensor fusion
    
    x = Ad*x + Bd*u(k);
    P = Ad*P*Ad' + Gd*Q_dyn_m(:,:,k)*Gd'; %Gd*Q*Gd';

end
disp('...finished!');

%% Plot

figure('Name','Real flight vs estimation Self calculated System 5');
plot(TimeVec,h);
grid on;
hold on;
plot(TimeVec,x_est_loop5(1,:)); 
plot(TimeVec,v);
plot(TimeVec,x_est_loop5(2,:));
plot(TimeVec,a);
plot(TimeVec,x_est_loop5(3,:));
plot(TimeVec,x_est_loop5(4,:));
hold off;
legend('real Height','estiamted Height','real Speed','estimated Speed','real acceloration','estimated acceloration','acceloration offset');
ylabel('height & speed & accelaration & pressure');
xlabel('Time [s]');

%% Loop 6 Extended Kalmanfilter
% Initalize the System
disp('System 6 with x1 = Height,x2 = Velocity,x3 = Acceloration,x4 = aoffset x5 = Pressure Lowpass Pressure for height calculation')
% x1 = Height, x2 = Velocity,x3 = Acceloration,x4 = aoffset x5 = Pressure
% x6 = Temp, x7 = Temp_dot
% Tempteratur at start = 15 C° -> 288.15 K°

A = [0 1 0 0 0 ;                %Height
    0 0 1 0 0 ;                 %Speed
    0 0 0 0 0 ;                 %Acceleration 
    0 0 0 0 0 ;                 %Acceleration Offset
    0 PressLookUp(2,1) 0 0 0];  %Pressure

B = [0;0;0;0;0];                %No direct input

C = [1 0 0 0 0;                 %GPS
    0 0 1 1 0;                  %IMU Accelerometer
    0 0 0 0 1;                  %Barometer 1
    0 0 0 0 1;                  %Barometer 2
    1 0 0 0 0 ];                %Height from the Pressure

D = [0];

G = [0 0 0 ;
    0 0 0 ;
    1 0 0 ;
    0 1 0 ;
    0 0 1 ];                    %System noise on acceloration, acc offset, pressure an Tdot

Sys = ss(A,B,C,D,Tau)

Ad = expm(A*Tau);
Gd = Ad * G;
Bd = Ad * B;

disp(['Size of A: ' num2str(size(A))]);

%% Observability test;
disp(['Rank: ' num2str(rank([C;C*Ad;C*Ad*Ad]))]);

%% Add Offset to system noise
%Static System noise:
%HGT = 0;
%SPE = 0;
ACEL = 70;
ACELOFS = 0.0001;
PRE = 0.1;
%TMP = 0;
%DTMP = 0.1;
Q = diag([ACEL;ACELOFS;PRE]);%;TMP;DTMP]);

%Dynamic Sytem noise:
%HGT = ones(1,length(TimeVec))*0;
%SPE = ones(1,length(TimeVec))*0;
ACEL = [100 100 100 50 30 ones(1,length(TimeVec)-5)*.01];
ACELOFS = ones(1,length(TimeVec))*ACELOFS;
%PRE = [zeros(1,10) ones(1,length(TimeVec)-20)*0.7 zeros(1,10)];         %Due to linearization it gets less accurate in the middle
PRE = ones(1,length(TimeVec))*0.001; 
%TMP = ones(1,length(TimeVec))*0;
%DTMP = ones(1,length(TimeVec))*0.1;
%Q_dyn = [HGT;SPE;ACEL;PRE;TMP;DTMP]; 
Q_dyn = [ACEL;ACELOFS;PRE];%;DTMP];

%Add all noise vectors into an noise matrix
Q_dyn_m = diag(Q_dyn(:,1)');
for n = 2:length(TimeVec)
    Q_dyn_m = cat(3,Q_dyn_m,diag(Q_dyn(:,n)'));
end

HEIGHTBARvar = (ones(1,length(p))+(p-P2))*0.2174;%sqrt(var(x_est_loop6(5,:)-p));
%Add all noise vectors into an noise matrix
R_dyn = [GPSvar;ACLvar;BM1var;BM2var;HEIGHTBARvar];
R_dyn_m = diag(R_dyn(:,1)');
for n = 2:length(TimeVec)
    R_dyn_m = cat(3,R_dyn_m,diag(R_dyn(:,n)'));
end
R_dyn_t = timeseries(R_dyn,TimeVec);

%% Do estimation Loop

u = zeros(1,length(TimeVec));                       %Input vector is zero
y = [h_mes_GPS;a_mes;p_mes_1;p_mes_2];%T_mes];      %Output are the measurements
x = [0;0;0;0;Po];%T(1);0];                          %Start Vector should be like this
deltaX = zeros(5,length(TimeVec));
P = eye(5);                                         %Standart can maybe be increased
x_est_loop6 = zeros(size(x,1),length(TimeVec));     %Vector for the SE values
disp('Loop 6 start..');
for k = 1:length(TimeVec)
    K = P*C'*pinv(C*P*C' + R_dyn_m(:,:,k));
    Height1 = CalcHeight(Po,x(5),T0,0,true);        %Calculate the height out of the pressure state vector
    y = [h_mes_GPS(k);a_mes(k);p_mes_1(k);p_mes_2(k);Height1];
    deltaX(:,k) = K*(y - C*x);
    x = x + K*(y - C*x);
    P = (eye(5)-K*C)*P;
    
    x_est_loop6(:,k) = x;                            %Save data from the Sensor fusion
    
    x = Ad*x + Bd*u(k);
    P = Ad*P*Ad' + Gd* Q_dyn_m(:,:,k)*Gd';
end
disp('...finished!');

%% Plot Loop 6

figure('Name','Real flight 2 vs estimation Self calculated');
plot(TimeVec,h);
grid on;
hold on;
plot(TimeVec,x_est_loop6(1,:)); 
plot(TimeVec,x_est_loop(1,:));
plot(TimeVec,v);
plot(TimeVec,x_est_loop6(2,:));
plot(TimeVec,a);
plot(TimeVec,x_est_loop6(3,:));
plot(TimeVec,x_est_loop6(4,:));
plot(TimeVec,p);
plot(TimeVec,x_est_loop6(5,:));
hold off;
legend('real Height','estiamted Height','estimated Height Simulink','real Speed','estimated Speed','real acceloration','estimated acceloration','acceleration offset','real pressure','estimated pressure');
ylabel('height & speed & accelaration & pressure');
xlabel('Time [s]');

%% System 7 Acceleration as Input LowPass Presure

A = [0 1 0 0 0;
     0 0 1 0 0;
     0 0 0 0 0;
     0 0 0 0 0;
     0 PressLookUp(2,1) 0 0 0];

Bd = [0;0;1;0;0];

C = [1 0 0 0 0;
     0 0 0 0 1;
     0 0 0 0 1;
     1 0 0 0 0];
 
G = [0 0 0 ;
    0 0 0 ;
    1 0 0 ;
    0 1 0 ;
    0 0 1 ];

Ad = [1 Tau 0 0 0;
      0 1 Tau 0 0;
      0 0 0 -1 0;
      0 0 0 1 0;
      0 PressLookUp(2,1)*Tau 0 0 1]
 
Gd = Ad*G
Cd = C*Ad

%% Make noise Matrices
%Static System noise:
%HGT = 0;
%SPE = 0;
ACEL = 70;
ACELOFS = 0.0001;
PRE = 0.1;
%TMP = 0;
%DTMP = 0.1;
Q = diag([ACEL;ACELOFS;PRE]);%;TMP;DTMP]);

%Dynamic Sytem noise:
%HGT = ones(1,length(TimeVec))*0;
%SPE = ones(1,length(TimeVec))*0;
ACEL = [100 100 100 50 30 ones(1,length(TimeVec)-5)*.01];
ACELOFS = ones(1,length(TimeVec))*ACELOFS;
%PRE = [zeros(1,10) ones(1,length(TimeVec)-20)*0.7 zeros(1,10)];         %Due to linearization it gets less accurate in the middle
PRE = ones(1,length(TimeVec))*0.001; 
%TMP = ones(1,length(TimeVec))*0;
%DTMP = ones(1,length(TimeVec))*0.1;
%Q_dyn = [HGT;SPE;ACEL;PRE;TMP;DTMP]; 
Q_dyn = [ACLvar;ACELOFS;PRE];%;DTMP];

%Add all noise vectors into an noise matrix
Q_dyn_m = diag(Q_dyn(:,1)');
for n = 2:length(TimeVec)
    Q_dyn_m = cat(3,Q_dyn_m,diag(Q_dyn(:,n)'));
end

HEIGHTBARvar = (ones(1,length(p))+(p-P2))*0.2174;%sqrt(10.0329/1.34);
%Add all noise vectors into an noise matrix
R_dyn = [GPSvar;BM1var;BM2var;HEIGHTBARvar];
R_dyn_m = diag(R_dyn(:,1)');
for n = 2:length(TimeVec)
    R_dyn_m = cat(3,R_dyn_m,diag(R_dyn(:,n)'));
end
R_dyn_t = timeseries(R_dyn,TimeVec);



%% Do estimation Loop

u = a_mes;                                          %Input vector is the acceloration
y = [h_mes_GPS;p_mes_1;p_mes_2];%T_mes];            %Output are the measurements
x = [0;0;0;0;Po];%T(1);0];                          %Start Vector should be like this
deltaX = zeros(5,length(TimeVec));
P = eye(5);                                         %Standart can maybe be increased
x_est_loop7 = zeros(size(x,1),length(TimeVec));     %Vector for the SE values
disp('Loop 7 start..');
for k = 1:length(TimeVec)
    K = P*C'*pinv(C*P*C' + R_dyn_m(:,:,k));
    Height1 = CalcHeight(Po,x(5),T0,0,true);        %Calculate the height out of the pressure state vector
    y = [h_mes_GPS(k);p_mes_1(k);p_mes_2(k);Height1];
    deltaX(:,k) = K*(y - C*x);
    x = x + K*(y - C*x);
    P = (eye(5)-K*C)*P;
    
    x_est_loop7(:,k) = x;                            %Save data from the Sensor fusion
    
    x = Ad*x + Bd*u(k);
    P = Ad*P*Ad' + Gd* Q_dyn_m(:,:,k)*Gd';
end
disp('...finished!');


%% Plot
figure('Name','Real flight 7 vs estimation Self calculated');
plot(TimeVec,h);
grid on;
hold on;
plot(TimeVec,x_est_loop7(1,:)); 
plot(TimeVec,x_est_loop(1,:));
plot(TimeVec,v);
plot(TimeVec,x_est_loop7(2,:));
plot(TimeVec,a);
plot(TimeVec,x_est_loop7(3,:));
plot(TimeVec,x_est_loop7(4,:));
plot(TimeVec,p);
plot(TimeVec,x_est_loop7(5,:));
hold off;
legend('real Height','estiamted Height','estimated Height Simulink','real Speed','estimated Speed','real acceloration','estimated acceloration','acceleration offset','real pressure','estimated pressure');
ylabel('height & speed & accelaration & pressure');
xlabel('Time [s]');


%% System 8 Acceleration as Input 

A = [0 1 0 0 0;
     0 0 1 0 0;
     0 0 0 0 0;
     0 0 0 0 0;
     0 PressLookUp(2,1) 0 0 0];

Bd = [0;0;1;0;];

C = [1 0 0 0 ;
     1 0 0 0 ;
     1 0 0 0 ];
 
G = [0 0;
    0 0;
    1 0;
    0 1];

Ad = [1 Tau 0 0 ;
      0 1 Tau 0 ;
      0 0 0 -1 ;
      0 0 0 1 ];
 
Gd = Ad*G
Cd = C*Ad

%% Make noise Matrices
%Static System noise:
%HGT = 0;
%SPE = 0;
ACEL = 70;
ACELOFS = 0.0001;
PRE = 0.1;
%TMP = 0;
%DTMP = 0.1;
Q = diag([ACEL;ACELOFS]);%;TMP;DTMP]);

%Dynamic Sytem noise:
%HGT = ones(1,length(TimeVec))*0;
%SPE = ones(1,length(TimeVec))*0;
ACEL = [100 100 100 50 30 ones(1,length(TimeVec)-5)*.01];
ACELOFS = ones(1,length(TimeVec))*ACELOFS;
%PRE = [zeros(1,10) ones(1,length(TimeVec)-20)*0.7 zeros(1,10)];         %Due to linearization it gets less accurate in the middle
PRE = ones(1,length(TimeVec))*0.001; 
%TMP = ones(1,length(TimeVec))*0;
%DTMP = ones(1,length(TimeVec))*0.1;
%Q_dyn = [HGT;SPE;ACEL;PRE;TMP;DTMP]; 
Q_dyn = [ACLvar;ACELOFS];%;DTMP];

%Add all noise vectors into an noise matrix
Q_dyn_m = diag(Q_dyn(:,1)');
for n = 2:length(TimeVec)
    Q_dyn_m = cat(3,Q_dyn_m,diag(Q_dyn(:,n)'));
end

%Add all noise vectors into an noise matrix
R_dyn = [GPSvar;BM1var;BM2var];
R_dyn_m = diag(R_dyn(:,1)');
for n = 2:length(TimeVec)
    R_dyn_m = cat(3,R_dyn_m,diag(R_dyn(:,n)'));
end
R_dyn_t = timeseries(R_dyn,TimeVec);



%% Loop initialization and loop

u = a_mes;                                       %Input vector is zero
x = [0;0;0;0];                                   %Start Vector should be like this
P = eye(4);                                      %Standart can maybe be increased
Height1 = 0;
Height2 = 0;
Temp = T(1);


x_est_loop5 = zeros(size(x,1),length(TimeVec));      %Vector for the SE values
disp('Loop 8 start..');
for k = 1:length(TimeVec)
    K = P*C'*pinv(C*P*C' + R_dyn_m(:,:,k));
    Height1 = CalcHeight(Po,p_mes_1(k),T0,0,true);
    Height2 = CalcHeight(Po,p_mes_2(k),T0,0,true);
    Temp = T1_mes(k);
    x = x + K*([h_mes_GPS(k);Height1;Height2]- C*x);
    P = (eye(4)-K*C)*P;
    
    x_est_loop8(:,k) = x;                            %Save data from the Sensor fusion
    
    x = Ad*x + Bd*u(k);
    P = Ad*P*Ad' + Gd*Q_dyn_m(:,:,k)*Gd'; %Gd*Q*Gd';

end
disp('...finished!');

%% Plot
figure('Name','Real flight 8 vs estimation Self calculated');
plot(TimeVec,h);
grid on;
hold on;
plot(TimeVec,x_est_loop8(1,:)); 
plot(TimeVec,x_est_loop(1,:));
plot(TimeVec,v);
plot(TimeVec,x_est_loop8(2,:));
plot(TimeVec,a);
plot(TimeVec,x_est_loop8(3,:));
plot(TimeVec,x_est_loop8(4,:));
hold off;
legend('real Height','estiamted Height','estimated Height Simulink','real Speed','estimated Speed','real acceloration','estimated acceloration','acceleration offset');
ylabel('height & speed & accelaration & pressure');
xlabel('Time [s]');


%% Differences between estimation and ground truth

figure('Name','Histogramm of the height Errors System 1-4');
diff = abs(h-x_est_loop(1,:));
subplot(4,1,1);
histogram(diff);
title('System 1: x=[h;v;a;p;T;Tdot]');
display(['Loop 1 "Normal" Max difference:' num2str(max(diff)) ' min difference:' num2str(min(diff)) ' mean:' num2str(mean(diff)) ' median: ' num2str(median(diff))]);
diff = abs(h-x_est_loop2(1,:));
subplot(4,1,2);
histogram(diff);
title('System 2: x=[h,v,a,aoffset,p,T,Tdot]');
display(['Loop 2 Acceloration with offset Max difference:' num2str(max(diff)) ' min difference:' num2str(min(diff)) ' mean:' num2str(mean(diff)) ' median: ' num2str(median(diff))]);
diff = abs(h-x_est_loop3(1,:));
subplot(4,1,3);
histogram(diff);
title('System 3: x=[h,v,a,aoffset,T]');
display(['Loop 3 Pressure as height Max difference:' num2str(max(diff)) ' min difference:' num2str(min(diff)) ' mean:' num2str(mean(diff)) ' median: ' num2str(median(diff))]);
diff = abs(h-x_est_loop4(1,:));
subplot(4,1,4);
histogram(diff);
title('System 4: x=[h,v,a,aoffset]');
display(['Loop 4 Pressure as height Max difference:' num2str(max(diff)) ' min difference:' num2str(min(diff)) ' mean:' num2str(mean(diff)) ' median: ' num2str(median(diff))]);

figure('Name','Histogramm of the height Errors System 5-8');
diff = abs(h-x_est_loop5(1,:));
subplot(4,1,1);
histogram(diff);
title('System 5: x=[h,v,a,aoffset] without GPS Measurements !');
display(['Loop 5 Without GPS Max difference:' num2str(max(diff)) ' min difference:' num2str(min(diff)) ' mean:' num2str(mean(diff)) ' median: ' num2str(median(diff))]);
diff = abs(h-x_est_loop6(1,:));
subplot(4,1,2);
histogram(diff);
title('System 6: x=[h,v,a,aoffset,p] with height depending on Pressure!');
display(['Loop 6 With lowPass Presure Max difference:' num2str(max(diff)) ' min difference:' num2str(min(diff)) ' mean:' num2str(mean(diff)) ' median: ' num2str(median(diff))]);
diff = abs(h-x_est_loop7(1,:));
subplot(4,1,3);
histogram(diff);
title('System 7: x=[h,v,a,aoffset,p] with height depending on Pressure and Acceloration as input!');
display(['Loop 7 With lowPass Presure and acceleration as input:, Max Difference:' num2str(max(diff)) ' min difference:' num2str(min(diff)) ' mean:' num2str(mean(diff)) ' median: ' num2str(median(diff))]);
diff = abs(h-x_est_loop8(1,:));
subplot(4,1,4);
histogram(diff);
title('System 8: x=[h,v,a,aoffset] with height depending on Pressure and Acceloration as input!');
display(['Loop 8 Pressure as height acceloration as input Max difference:' num2str(max(diff)) ' min difference:' num2str(min(diff)) ' mean:' num2str(mean(diff)) ' median: ' num2str(median(diff))]);

figure('Name','Plot of the absolute height Errors System 1-6');
hold on;
diff = abs(h-x_est_loop(1,:));
plot(TimeVec,diff);
diff = abs(h-x_est_loop2(1,:));
plot(TimeVec,diff);
diff = abs(h-x_est_loop3(1,:));
plot(TimeVec,diff);
diff = abs(h-x_est_loop4(1,:));
plot(TimeVec,diff);
diff = abs(h-x_est_loop5(1,:));
plot(TimeVec,diff);
diff = abs(h-x_est_loop6(1,:));
plot(TimeVec,diff);
diff = abs(h-x_est_loop7(1,:));
plot(TimeVec,diff);
diff = abs(h-x_est_loop8(1,:));
plot(TimeVec,diff);
legend('x = [h;v;a;p;T;Tdot] Pressure not influencing height','x=[h,v,a,aoffset,p,T,Tdot] Pressure not influencing height','x=[h,v,a,aoffset,T]','x=[h,v,a,aoffset]','x=[h,v,a,aoffset] Without GPS','x=[h,v,a,aoffset,p] LowPass Pressure','x=[h,v,a,aoffset,p] a as input','x=[h,v,a,aoffset] a as input');
hold off;
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
%%
figure('Name','Pressure difference')
for k = 1:length(TimeVec)
    h6(k) = CalcHeight(x_est_loop6(5,1),x_est_loop6(5,k),T0,0,true);
end
h6est2 = x_est_loop6(1,:);

plot(abs(p-x_est_loop6(5,:)));
hold on;
plot(abs(p-p_mes_1));
legend('with press as height','pressure lowpass filtered')
median(abs(p-x_est_loop6(5,:)))
median(abs(p-p_mes_1))