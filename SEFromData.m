% In this script the State Estimation from given Sensor Data or self produced should be
% compared to the ground truth.

clear all; close all;
%% Important variable definition

Tgrad = 0.0065;         %Temperatur gradient to generate pressure data
TgradSimu = 0.0065;     %Temperatur gradient which is used to calculate the height during state estimation
GPSTau = 1;             %Sampling tau GPS
P1Tau = 0.02;           %Sampling tau first Barometer
P2Tau = 0.01;           %Sampling tau second Barometer
usePerfData = false;    %Make simulation with perfect or realistic sensor data

%% Load TimeVec and Trajectory from Hassans Simulation

%load('StateSimu.mat')
load('RoroState.mat')

TimeVec = t';
Tau = TimeVec(end)/length(TimeVec);
h = state(:,3)';

% plot it
figure('Name','Real Data')
plot(TimeVec,h,'b')             %Original trajectory
legend('Real height in z');


%% Produce noisless sensor Data self

disp('Generate sensor data...')
figure('Name','Real Data');
% get acceloration by differentiate height:
v = diff(h)/(Tau);
%a = diff(v)/(Tau);
% Ad zeros to maintain vector length
v = [v v(end)];
%a = [a a(end) a(end)];
%anew = resample(imu_a(:,3),imu_t,1000);
%a = anew(1:length(h))';
a = diff(v)/(Tau);
a = [a a(end)];

% Find burnout index
k = 20;
while a(k) > 20
    k = k+1;
end
T_brn_ind = k;

phi = zeros(1,length(TimeVec));
% Integrate over more or less random values to get a pitch angle
% representet with air current etc
for k = 2:length(TimeVec)  
    if k > T_brn_ind
       phi(k) = phi(k-1) + (randn(1,1)+0.005) * 400 * Tau ;
    else
       phi(k) = phi(k-1) + randn(1,1) * 20 * Tau; 
    end 
end
% Filter the noise phi angle with a moving average filter
n = 10000;
phi = filter((1/n)*ones(1,n),1,phi);

aphi = 1./cos(phi*pi/180) .* a;

% get height of GPS by deleting engouh values so it becomes 5Hz sample rate
% and then ad Zero Order Hold to get static value
%GPSTau = 1;
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
%Tgrad = 0.00649;
T0 = 15 + 273.15;
T = T0 - Tgrad*h;

% Get Barometric Data (pressure)
% Pressure Data Temp/Po are just assumptions !!!!
Po = 1013.25;    %Pressure at altitude 0

p = Po*(1-(Tgrad*h)./T0).^((0.02896*9.807)/(8.314*Tgrad));

%P1 samples slower
%P1Tau = 1/50;
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
                            
%P2 samples faster
%P2Tau = 1/100;
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


plot(TimeVec,h);
hold on;
plot(TimeVec,h_GPS)
plot(TimeVec,a);
plot(TimeVec,aphi);
plot(TimeVec,T)
plot(TimeVec,p);
plot(TimeVec,P1);
plot(TimeVec,P2);
plot(TimeVec,phi);
legend('Real height in z','GPS heigt in z','Real Acceloration','Acceleration wiht pitch angle','Assumed Temperature in Kelvin','Real Pressure','Assumed Pressure 1','Assumed Pressure 2','Pitch angle');
hold off;
disp('...finished!');

%% Add noise to sensor data
disp('Add noise onto sensor data...')

% Load the AR models of the different noises
load('h_a.mat');

% T_var_brn = 8.4040e-04;
% p_var_brn = 1.7034;
%p2_var_brn = p_var_brn * 2;
% p_var_upflight = 0.7034;
%p2_var_upflight = p_var_upflight * 2;
% a_var_brn = 0.0128;
% a_var_upflight = 0.0028;
%GPS_var = 0.1;

if usePerfData == true
    
    % Simulation with optimal data
    T1_mes = T;
    T2_mes = T;
    h_mes_GPS = h_GPS;
    p_mes_1 = P1;
    p_mes_2 = P2;
    a_mes = a;
    phi_mes = zeros(1,length(TimeVec));
    
else
    
    % Acceleloration > Add a offset and more noise if while the motor is
    % burning
    a_mes = zeros(1,length(a));
    a_offset = 4;
    aofst_var_brn = 0.00001;
    aofst_var_upflight = 0.00001;
    % Generate noiese Vecotrs:
    a_noise_brn = filter(1,h_brn,randn(1,T_brn_ind) * sqrt(varBrn));
    aofst_noise_brn = filter(1,h_brn,randn(1,T_brn_ind) * sqrt(aofst_var_brn));
    aofst_noise_brn = filter(ones(1,1000)*1/1000,1,aofst_noise_brn);
    a_noise_upflight = filter(1,h_upflight,randn(1,length(a)-T_brn_ind) * sqrt(varUpflight));
    aofst_noise_upflight = filter(1,h_upflight,randn(1,length(a)-T_brn_ind) * sqrt(aofst_var_upflight));
    aofst_noise_upflight = filter(ones(1,1000)*1/1000,1,aofst_noise_upflight);
    for k = 1:length(a)
         if k <= T_brn_ind
         a_mes(k) = aphi(k) + a_noise_brn(k) + a_offset + aofst_noise_brn(k);
         else
         a_mes(k) = aphi(k) + a_noise_upflight(k-T_brn_ind) + a_offset + aofst_noise_upflight(k-T_brn_ind);    
         end
    end
    ACELOFS2 = [aofst_noise_brn aofst_noise_upflight];
    % delete the loaded data from workspace
    clearvars a_noise_brn a_noise_upflight aofst_noise_brn aofst_noise_upflight aofst_var_brn aofst_var_upflight


    % Pitch angle, using the same noise as acceloration cause it should have
    % more or less the sam properties -> Both IMU 
    % Maybe change this in a later implementation
    load('h_phi.mat');
    phi_noise_brn = filter(1,h_brn,randn(1,T_brn_ind) * sqrt(varBrn));
    phi_noise_upflight = filter(1,h_upflight,randn(1,length(a)-T_brn_ind) * sqrt(varUpflight));
    for k = 1:length(a)
        if k <= T_brn_ind
            phi_mes(k) = phi(k) + phi_noise_brn(k);
        else
            phi_mes(k) = phi(k) + phi_noise_upflight(k-T_brn_ind);
        end
    end
    clearvars phi_noise_brn phi_noise_upflight

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
    clearvars p_noise_brn p_noise_upflight p2_noise_brn p2_noise_upflight p2_var_brn p2_var_upflight

    % GPS
    load('h_GPS.mat');
    h_mes_GPS = h_GPS + filter(1,h_flight,randn(1,length(h_GPS)).*sqrt(varFlight));

    
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
    clearvars T1_noise_brn T1_noise_upflight T2_noise_brn T2_noise_upflight

    clearvars varBrn varFlight varPreIco varUpflight h_brn h_preIco h_upflight h_flight
    %load('h_phi.mat')
    %T_mes = T + randn(1,length(T)).*sqrt(T_var_brn);

    % T_mes = awgn(T,40,'measured');
    % h_mes_GPS = awgn(h_GPS,80,'measured');
    % p_mes_1 = awgn(p,45,'measured');
    % p_mes_2 = awgn(p,40,'measured');
    % a_mes = awgn(a,30,'measured');

end
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
disp('...finished!');

%% Create Noise Matrices
disp('Create noise matrices...');
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

%Variance of the measurements during burn
GPSvar_brn = var(h_mes_GPS(1:T_brn_ind)-h_GPS(1:T_brn_ind));
ACLvar_brn = var(a_mes(1:T_brn_ind)-a(1:T_brn_ind));
BM1var_brn = var(p_mes_1(1:T_brn_ind)-P1(1:T_brn_ind));
BM2var_brn = var(p_mes_2(1:T_brn_ind)-P2(1:T_brn_ind));
TRMvar_brn = var(T1_mes(1:T_brn_ind)-T1(1:T_brn_ind));
PHIvar_brn = var(phi_mes(1:T_brn_ind)-phi(1:T_brn_ind));

%Variance of the measurements after the burnout
GPSvar_upflight = var(h_mes_GPS(T_brn_ind:end)-h_GPS(T_brn_ind:end));
ACLvar_upflight = var(a_mes(T_brn_ind:end)-a(T_brn_ind:end));
BM1var_upflight = var(p_mes_1(T_brn_ind:end)-P1(T_brn_ind:end));
BM2var_upflight = var(p_mes_2(T_brn_ind:end)-P2(T_brn_ind:end));
TRMvar_upflight = var(T1_mes(T_brn_ind:end)-T1(T_brn_ind:end));
PHIvar_upflight = var(phi_mes(T_brn_ind:end)-phi(T_brn_ind:end));

%Dynamic Sensor noise:
GPSStep = [GPSvar ones(1,round(GPSTau/Tau) - 1)*2^32];
GPSvar = [];
for n = 1:round(length(TimeVec)/length(GPSStep))-1
    GPSvar = [GPSvar GPSStep];
end
GPSvar = [GPSvar ones(1,length(TimeVec)-length(GPSvar))*2^32];

ACLvar = [ones(1,length(TimeVec(1:T_brn_ind)))*ACLvar_brn ones(1,length(TimeVec(T_brn_ind:end))-1)*ACLvar_upflight];

PHIvar = [ones(1,length(TimeVec(1:T_brn_ind)))*PHIvar_brn ones(1,length(TimeVec(T_brn_ind:end))-1)*PHIvar_upflight];

BM1Step = [BM1var_brn ones(1,round(P1Tau/Tau)-1)*2^32];
BM1var = [];
for n = 1:round(length(TimeVec)/length(BM1Step))-1
    if length(BM1var) > T_brn_ind
        BM1Step = [BM1var_upflight ones(1,round(P1Tau/Tau)-1)*2^32];
    end
    BM1var = [BM1var BM1Step];
end
BM1var = [BM1var ones(1,length(TimeVec)-length(BM1var))*2^32];
%BM1var = ones(1,length(TimeVec))*0.5;

BM2Step = [BM2var_brn ones(1,round(P2Tau/Tau)-1)*2^32];
BM2var = [];
for n = 1:round(length(TimeVec)/length(BM2Step))-1
    if length(BM2var) > T_brn_ind
        BM2Step = [BM2var_upflight ones(1,round(P2Tau/Tau)-1)*2^32];
    end
    BM2var = [BM2var BM2Step];
end
BM2var = [BM2var ones(1,length(TimeVec)-length(BM2var))*2^32];
%BM2var = ones(1,length(TimeVec))*0.3;

TRMvar = [ones(1,length(TimeVec(1:T_brn_ind)))*TRMvar_brn ones(1,length(TimeVec(T_brn_ind:end))-1)*TRMvar_upflight];;

%Add all noise vectors into an noise matrix
R_dyn = [GPSvar;ACLvar;BM1var;BM2var;TRMvar];
R_dyn_m = diag(R_dyn(:,1)');
for n = 2:length(TimeVec)
    R_dyn_m = cat(3,R_dyn_m,diag(R_dyn(:,n)'));
end
R_dyn_t = timeseries(R_dyn,TimeVec);


%% System Noise

%Static System noise:
HGT = 0;
SPE = 0;
ACEL = 70;
PHI = 0.0005;
PRE = 0.1;
TMP = 0;
DTMP = 0.1;
Q = diag([HGT;SPE;ACEL;PRE;TMP;DTMP]);

%Dynamic Sytem noise:
HGT = ones(1,length(TimeVec))*0;
SPE = ones(1,length(TimeVec))*0;
ACEL = [100 100 100 50 30 ones(1,length(TimeVec)-5)*0.01];
PHI = [ones(1,length(TimeVec))*PHI];
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
disp('...finished!');

%% Initalize the System 1
disp('System 1 with x1 = Height,x2 = Velocity,x3 = Acceloration,x4 = Pressure,x5 = Temp,6 = Temp_dot');

% x1 = Height, x2 = Velocity,x3 = Acceloration, x4 = Pressure, x5 = Temp,
% x6 = Temp_dot

load('PressLookUp.mat') % Load the coefficient from the linearization

A = [0 1 0 0 0 0;                                   %Height
     0 0 1 0 0 0;                                    %Speed
     0 0 0 0 0 0;                                    %Acceleration
     0 PressLookUp(2,1) 0 0 0 PressLookUp(1,1);      %Pressure
     0 0 0 0 0 1;                                    %Temperature
     0 0 0 0 0 0];                                   %Temperature dot

B = [0;0;0;0;0;0];                                  %No direct input

C = [1 0 0 0 0 0;
     0 0 1 0 0 0;
     0 0 0 1 0 0;
     0 0 0 1 0 0;
     0 0 0 0 1 0];

D = [0];

G = [0 0 0;
     0 0 0;
     1 0 0;
     0 1 0;
     0 0 0;
     0 0 1];                

% Just for the text message in the command window
%Sys = ss(A,B,C,D,Tau)

% Make the matrices time discret
Ad = expm(A*Tau);     
Gd = Ad * G;
Bd = Ad * B;

disp(['Size of A: ' num2str(size(A))]);

%% Observability test;
disp(['Rank: ' num2str(rank([C;C*Ad;C*Ad*Ad]))]);

%% Initialize
u = zeros(1,length(TimeVec));                       %Input vector is zero
y = [h_mes_GPS;a_mes;p_mes_1;p_mes_2;T1_mes];       %Output are the measurements
y_t = timeseries(y,TimeVec);                        %Is needed by the simulink model
x0 = [0;0;0;Po;T(1);0];                             %Start points
x =  x0;                  
P0 = eye(6);                    

%% Excecute Simulation with simulink Kalmanfilter
if usePerfData == false
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
end
%% Loop with first System

%u = zeros(1,length(TimeVec));                       %Input vector is zero
%y = [h_mes_GPS;a_mes;p_mes_1;p_mes_2;T1_mes];        %Output are the measurements
x = [0;0;0;Po;T(1);0];                              %Start Vector should be like this
P = eye(6);                                         %Standart can maybe be increased

x_est_loop = zeros(size(x,1),length(TimeVec));      %Vector for the SE values
disp('Loop 1 start..');
for k = 1:length(TimeVec)
    K = P*C'*pinv(C*P*C' + R_dyn_m(:,:,k));
    x = x + K*(y(:,k) - C*x);
    P = (eye(6)-K*C)*P;
    
    x_est_loop(:,k) = x;                            %Save data from the Sensor fusion
    
    x = Ad*x; % + Bd*u(k);                          %Not needed due to none input 
    P = Ad*P*Ad' + Gd* Q_dyn_m(:,:,k)*Gd';

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
legend('real Height','estiamted Height','real Speed','estimated Speed','real acceloration','estimated acceloration','real pressure','estimated pressure');
ylabel('height & speed & accelaration & pressure');
xlabel('Time [s]');



%% System 2
% Initalize the System
disp('System 2 with x1 = Height,x2 = Velocity,x3 = Acceloration,x4 = aoffset x5 = Pressure,x6 = Temp,x7 = Temp_dot')

% x1 = Height, x2 = Velocity,x3 = Acceloration,x4 = aoffset x5 = Pressure
% x6 = Temp, x7 = Temp_dot

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
    0 1];% 0 0;
    %0 0 1 0;
    %0 0 0 0;
    %0 0 0 1];                %System noise on acceloration, acc offset, pressure an Tdot

%Sys = ss(A,B,C,D,Tau)

% Make the matrices time discret
Ad = expm(A*Tau);     
Gd = Ad * G;
Bd = Ad * B;

disp(['Size of A: ' num2str(size(A))]);

%% Observability test;
disp(['Rank: ' num2str(rank([C;C*Ad;C*Ad*Ad]))]);

%% Adjust noise matrices for this system

%Static System noise:
HGT = 0;
SPE = 0;
ACEL = 70;
ACELOFS = 0.00001;
PRE = 0.1;
TMP = 0;
DTMP = 0.1;
Q = diag([ACEL;ACELOFS;PRE;TMP;DTMP]);

%Dynamic Sytem noise:
HGT = ones(1,length(TimeVec))*0;
SPE = ones(1,length(TimeVec))*0;
%ACEL = [100 100 100 50 30 ones(1,length(TimeVec)-5)*.1];
ACEL = diff(a);
ACEL = abs(ACEL);
ACEL = filter(ones(1,100)*1/100,1,ACEL);
ACEL = [ACEL 0];
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
x = [0;0;0;0];%;Po;T(1);0];                            %Start Vector should be like this
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
legend('real Height','estiamted Height','estimated Height System 1','real Speed','estimated Speed','real acceloration','estimated acceloration','acceleration offset','real pressure','estimated pressure');
ylabel('height & speed & accelaration & pressure');
xlabel('Time [s]');


%% Loop 3
disp('System 3 with x1=Height,x2=Speed,x3=Acceleration,x4=Acceleration Offset,x5=Phi');
% Initialize the system

A = [0 1 0 0 0;
    0 0 1 0 0;
    0 0 0 0 0;
    0 0 0 0 0;
    0 0 0 0 0];
B = [0;0;0;0;0];                %No direct input
C = [1 0 0 0 0;
    0 0 1 -1 0;
    1 0 0 0 0;
    1 0 0 0 0;
    0 0 0 0 1];          
D = [0];
G = [0 0 0;
    0 0 0;
    1 0 0;
    0 1 0;
    0 0 1];                %System noise only on acceloration


 %Diskretierung der Systemmatritzen
Ad = expm(A*Tau);     
Gd = Ad * G;
Bd = Ad * B;

disp(['Size of A: ' num2str(size(A))]);

%% Observability test;
disp(['Rank: ' num2str(rank([C;C*Ad;C*Ad*Ad]))]);

%% System noise implementation

%Q_dyn = [HGT;SPE;ACEL;TMP];  
Q_dyn = [ACEL;ACELOFS;PHI];
%Add all noise vectors into an noise matrix
Q_dyn_m = diag(Q_dyn(:,1)');
for n = 2:length(TimeVec)
    Q_dyn_m = cat(3,Q_dyn_m,diag(Q_dyn(:,n)'));
end

%Add all noise vectors into an noise matrix
R_dyn = [GPSvar;ACLvar;BM1var;BM2var;PHIvar];
R_dyn_m = diag(R_dyn(:,1)');
for n = 2:length(TimeVec)
    R_dyn_m = cat(3,R_dyn_m,diag(R_dyn(:,n)'));
end
R_dyn_t = timeseries(R_dyn,TimeVec);


%% Loop initialization and loop

u = zeros(1,length(TimeVec));                       %Input vector is zero
y = [h_mes_GPS;a_mes;p_mes_1;p_mes_2;T1_mes];        %Output are the measurements
y_t = timeseries(y,TimeVec);
x = [0;0;0;0;0];                                   %Start Vector should be like this
P = eye(5);                                         %Standart can maybe be increased
Height1 = 0;
Height2 = 0;
Temp = T(1);


x_est_loop3 = zeros(size(x,1),length(TimeVec));      %Vector for the SE values
disp('Loop 3 start..');
for k = 1:length(TimeVec)
    K = P*C'*pinv(C*P*C' + R_dyn_m(:,:,k));
    Height1 = CalcHeight(Po,p_mes_1(k),T0,0,true,TgradSimu);
    Height2 = CalcHeight(Po,p_mes_2(k),T0,0,true,TgradSimu);
    acc = a_mes(k) * cos(x(5)*pi/180);
    %Temp = T1_mes(k);
    x = x + K*([h_mes_GPS(k);acc;Height1;Height2;phi_mes(k)] - C*x);
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
plot(TimeVec,phi);
plot(TimeVec,x_est_loop3(5,:));
hold off;
legend('real Height','estiamted Height','estimated Height System 1','real Speed','estimated Speed','real acceloration','estimated acceloration','acceloration offset','real pitch angle','estimated pitch angle');
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


% make the time depending matrices discret
Ad = expm(A*Tau);     
Gd = Ad * G;
Bd = Ad * B;

disp(['Size of A: ' num2str(size(A))]);

%% Observability test;
disp(['Rank: ' num2str(rank([C;C*Ad;C*Ad*Ad]))]);

%% System noise implementation

%Dynamic Sytem noise:
ACEL = [100 100 100 50 30 ones(1,length(TimeVec)-5)*0.01];

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
    Height1 = CalcHeight(Po,p_mes_1(k),T0,0,true,TgradSimu);
    Height2 = CalcHeight(Po,p_mes_2(k),T0,0,true,TgradSimu);
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
legend('real Height','estiamted Height','estimated Height System 1','real Speed','estimated Speed','real acceloration','estimated acceloration','acceloration offset');
ylabel('height & speed & accelaration & pressure');
xlabel('Time [s]');


%% Loop 5 Without Temperature and GPS

disp('System 5 with x1=Height,x2=Speed,x3=Acceleration,x4=Acceleration Offset and without GPS');
% Initialize the system

A = [0 1 0 0 ;              %Height
     0 0 1 0 ;              %Speed
     0 0 0 0 ;              %Acceleration
     0 0 0 0] ;             %Acceleration Offset

B = [0;0;0;0];

C = [0 0 1 -1;
     1 0 0 0;
     1 0 0 0];

D = [0];

G = [0 0;
    0 0;
    1 0;
    0 1];

%Sys = ss(A,B,C,D,Tau) 

% Make the matrices time discret
Ad = expm(A*Tau);     
Gd = Ad * G;
Bd = Ad * B;

disp(['Size of A: ' num2str(size(A))]);

%% Observability test;
disp(['Rank: ' num2str(rank([C;C*Ad;C*Ad*Ad]))]);

%% Adjust noise matrices for this sytem

%System noise
Q_dyn = [ACEL;ACELOFS];
%Add all noise vectors into an noise matrix
Q_dyn_m = diag(Q_dyn(:,1)');
for n = 2:length(TimeVec)
    Q_dyn_m = cat(3,Q_dyn_m,diag(Q_dyn(:,n)'));
end

%Masurment noise
%Add all noise vectors into an noise matrix
R_dyn = [ACLvar;BM1var;BM2var];
R_dyn_m = diag(R_dyn(:,1)');
for n = 2:length(TimeVec)
    R_dyn_m = cat(3,R_dyn_m,diag(R_dyn(:,n)'));
end
R_dyn_t = timeseries(R_dyn,TimeVec);


%% Loop initialization and loop

u = zeros(1,length(TimeVec));                       %Input vector is zero
x = [0;0;0;0];                                      %Start Vector should be like this
P = eye(4);                                         %Standart can maybe be increased
Height1 = 0;
Height2 = 0;
Temp = T(1);

x_est_loop5 = zeros(size(x,1),length(TimeVec));     %Vector for the SE values
disp('Loop 5 start..');
for k = 1:length(TimeVec)
    K = P*C'*pinv(C*P*C' + R_dyn_m(:,:,k));
    Height1 = CalcHeight(Po,p_mes_1(k),T0,0,true,TgradSimu);
    Height2 = CalcHeight(Po,p_mes_2(k),T0,0,true,TgradSimu);
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

%% System 6 Pressure LowPass filtered
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

%Sys = ss(A,B,C,D,Tau)

Ad = expm(A*Tau);
Gd = Ad * G;
Bd = Ad * B;

disp(['Size of A: ' num2str(size(A))]);

%% Observability test;
disp(['Rank: ' num2str(rank([C;C*Ad;C*Ad*Ad]))]);

%% Ajust noise matrices
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
    Height1 = CalcHeight(Po,x(5),T0,0,true,TgradSimu);        %Calculate the height out of the pressure state vector
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
legend('real Height','estiamted Height','estimated Height System 1','real Speed','estimated Speed','real acceloration','estimated acceloration','acceleration offset','real pressure','estimated pressure');
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
      0 PressLookUp(2,1)*Tau 0 0 1];
 
Gd = Ad*G;
Cd = C*Ad;

%% Adjust noise Matrices
%Static System noise:
ACEL = 70;
ACELOFS = 0.0001;
PRE = 0.1;
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
    Height1 = CalcHeight(Po,x(5),T0,0,true,TgradSimu);        %Calculate the height out of the pressure state vector
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
legend('real Height','estiamted Height','estimated Height System 1','real Speed','estimated Speed','real acceloration','estimated acceloration','acceleration offset','real pressure','estimated pressure');
ylabel('height & speed & accelaration & pressure');
xlabel('Time [s]');


%% System 8 Should get the best result 

disp('System 8 with x1=Height,x2=Speed,x3=Acceleration,x4=Acceleration Offset,x5=Phi');
% Initialize the system

A = [0 1 0 0 0;                 %Height
    0 0 1 0 0;                  %Speed
    0 0 0 0 0;                  %Acceleration
    0 0 0 0 0;                  %Acceleration offset
    0 0 0 0 0];                 %Phi

B = [0;0;0;0;0];                %No direct input

C = [1 0 0 0 0;
    0 0 1 1 0;
    1 0 0 0 0;
    1 0 0 0 0;
    0 0 0 0 1];

D = [0];

G = [0 0 0;
    0 0 0;
    1 0 0;                      
    0 1 0;
    0 0 1];                


% Make the system time discret
Ad = expm(A*Tau);     
Gd = Ad * G;
Bd = Ad * B;

disp(['Size of A: ' num2str(size(A))]);

%% Observability test;
disp(['Rank: ' num2str(rank([C;C*Ad;C*Ad*Ad]))]);

%% Adjust noise matrices

% Optimal system noises calculation by differating the corresponding vector
% Acceleration
ACEL = abs(diff(a));
ACEL = filter(ones(1,100)*1/100,1,ACEL);
ACEL = [ACEL 0];
% Acceleration offset
ACELOFS = abs(diff(ACELOFS2));
ACELOFS = filter(ones(1,1000)*1/1000,1,ACELOFS);
ACELOFS = [ACELOFS 0];
% Pitch angle
PHI = abs(diff(phi));
PHI = filter(ones(1,1000)*1/1000,1,PHI);
PHI = [PHI 0];

Q_dyn = [ACEL;ACELOFS;PHI];

%Add all noise vectors into an noise matrix
Q_dyn_m = diag(Q_dyn(:,1)');
for n = 2:length(TimeVec)
    Q_dyn_m = cat(3,Q_dyn_m,diag(Q_dyn(:,n)'));
end

%Add all noise vectors into an noise matrix
R_dyn = [GPSvar;ACLvar;BM1var;BM2var;PHIvar];
R_dyn_m = diag(R_dyn(:,1)');
for n = 2:length(TimeVec)
    R_dyn_m = cat(3,R_dyn_m,diag(R_dyn(:,n)'));
end
R_dyn_t = timeseries(R_dyn,TimeVec);


%% Loop initialization and loop

SensorOutfallTime = 100;
SensorOutfall = ['GPS'];
inf = 100000;

u = zeros(1,length(TimeVec));                       %Input vector is zero
y = [h_mes_GPS;a_mes;p_mes_1;p_mes_2;T1_mes];       %Output are the measurements
y_t = timeseries(y,TimeVec);
x = [0;0;a_mes(1);a_offset;0];                      %Start Vector should be like this
P = eye(5);                                         %Standart can maybe be increased
Height1 = 0;
Height2 = 0;

x_est_loop8 = zeros(size(x,1),length(TimeVec));      %Vector for the SE values

disp('Loop 8 start..');
for k = 1:length(TimeVec)

    if mod(TimeVec(k),GPSTau) == 0 %GPS measurement arrived
        if k > T_brn_ind
            GPSvarTemp = GPSvar_upflight;
        else
            GPSvarTemp = GPSvar_brn;
        end
    else
        GPSvarTemp = inf;
    end
    
    if mod(TimeVec(k),P1Tau) == 0 %Barometer 1 measurement arrived
        if k > T_brn_ind
            BM1varTemp = BM1var_upflight /9.801;
        else
            BM1varTemp = BM1var_brn /9.801;
        end
        Height1 = CalcHeight(Po,p_mes_1(k),T0,0,true,TgradSimu);
    else
        BM1varTemp = inf;
    end
    
    if mod(TimeVec(k),P2Tau) == 0 %Barometer 2 measurement arrived
        if k > T_brn_ind
            BM2varTemp = BM2var_upflight /9.801;
        else
            BM2varTemp = BM2var_brn /9.801;
        end
        Height2 = CalcHeight(Po,p_mes_2(k),T0,0,true,TgradSimu);
    else
        BM2varTemp = inf;
    end
    ACLvarTemp = ACLvar(k);
    PHIvarTemp = PHIvar(k);
    GPStemp = h_mes_GPS(k);
    ACLtemp = a_mes(k) * cos(x(5)*pi/180);
    PHItemp = phi_mes(k);
    
    if TimeVec(k) > SensorOutfallTime
        for i = 1:size(SensorOutfall)
            switch SensorOutfall(i,:)
                case 'GPS'
                    GPSvarTemp = inf;
                    GPStemp = 0;
                case 'Acc'
                    ACLvarTemp = inf;
                    ACLtemp = 0;
                case 'Phi'
                    PHIvarTemp = inf;
                    PHItemp = 0;
                case 'P_1'
                    BM1varTemp = inf;
                    Height1 = 0;
                case 'P_2'
                    BM2varTemp = inf;
                    Height2 = 0;
            end
        end
    end
    

    R = diag([GPSvarTemp;ACLvarTemp;BM1varTemp;BM2varTemp;PHIvarTemp]);
    K = P*C'*pinv(C*P*C' + R);%R_dyn_m(:,:,k));
    x = x + K*([GPStemp;ACLtemp;Height1;Height2;PHItemp] - C*x);
    P = (eye(5)-K*C)*P;
    
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
plot(TimeVec,phi);
plot(TimeVec,x_est_loop8(5,:));
hold off;
legend('real Height','estiamted Height','estimated Height System 1','real Speed','estimated Speed','real acceloration','estimated acceloration','acceleration offset','pitch angle','pitch angle estimate');
ylabel('height & speed & accelaration & pressure');
xlabel('Time [s]');


%% Differences between estimation and ground truth
close all;
figure('Name','Histogramm of the height Errors System 1-4');
difference = abs(h-x_est_loop(1,:));
subplot(4,1,1);
histogram(difference);
title('System 1: x=[h;v;a;p;T;Tdot]');
display(['Loop 1 "Normal" Max difference:' num2str(max(difference)) ' min difference:' num2str(min(difference)) ' mean:' num2str(mean(difference)) ' median: ' num2str(median(difference))]);
difference = abs(h-x_est_loop2(1,:));
subplot(4,1,2);
histogram(difference);
title('System 2: x=[h,v,a,aoffset,p,T,Tdot]');
display(['Loop 2 Acceloration with offset Max difference:' num2str(max(difference)) ' min difference:' num2str(min(difference)) ' mean:' num2str(mean(difference)) ' median: ' num2str(median(difference))]);
difference = abs(h-x_est_loop3(1,:));
subplot(4,1,3);
histogram(difference);
title('System 3: x=[h,v,a,aoffset,T]');
display(['Loop 3 Pressure as height Max difference:' num2str(max(difference)) ' min difference:' num2str(min(difference)) ' mean:' num2str(mean(difference)) ' median: ' num2str(median(difference))]);
difference = abs(h-x_est_loop4(1,:));
subplot(4,1,4);
histogram(difference);
title('System 4: x=[h,v,a,aoffset,phi]');
display(['Loop 4 Pressure as height Max difference:' num2str(max(difference)) ' min difference:' num2str(min(difference)) ' mean:' num2str(mean(difference)) ' median: ' num2str(median(difference))]);

figure('Name','Histogramm of the height Errors System 5-8');
difference = abs(h-x_est_loop5(1,:));
subplot(4,1,1);
histogram(difference);
title('System 5: x=[h,v,a,aoffset] without GPS Measurements !');
display(['Loop 5 Without GPS Max difference:' num2str(max(difference)) ' min difference:' num2str(min(difference)) ' mean:' num2str(mean(difference)) ' median: ' num2str(median(difference))]);
difference = abs(h-x_est_loop6(1,:));
subplot(4,1,2);
histogram(difference);
title('System 6: x=[h,v,a,aoffset,p] with height depending on Pressure!');
display(['Loop 6 With lowPass Presure Max difference:' num2str(max(difference)) ' min difference:' num2str(min(difference)) ' mean:' num2str(mean(difference)) ' median: ' num2str(median(difference))]);
difference = abs(h-x_est_loop7(1,:));
subplot(4,1,3);
histogram(difference);
title('System 7: x=[h,v,a,aoffset,p] with height depending on Pressure and Acceloration as input!');
display(['Loop 7 With lowPass Presure and acceleration as input:, Max Difference:' num2str(max(difference)) ' min difference:' num2str(min(difference)) ' mean:' num2str(mean(difference)) ' median: ' num2str(median(difference))]);
difference = abs(h-x_est_loop8(1,:));
subplot(4,1,4);
histogram(difference);
title('System 8: x=[h,v,a,aoffset,phi] with height depending on Pressure');
display(['Loop 8 Pressure as height and phi in vector Max difference:' num2str(max(difference)) ' min difference:' num2str(min(difference)) ' mean:' num2str(mean(difference)) ' median: ' num2str(median(difference))]);

figure('Name','Plot of the absolute height Errors System 1-8');
hold on;
difference = abs(h-x_est_loop(1,:));
plot(TimeVec,difference);
difference = abs(h-x_est_loop2(1,:));
plot(TimeVec,difference);
difference = abs(h-x_est_loop3(1,:));
plot(TimeVec,difference);
difference = abs(h-x_est_loop4(1,:));
plot(TimeVec,difference);
difference = abs(h-x_est_loop5(1,:));
plot(TimeVec,difference);
difference = abs(h-x_est_loop6(1,:));
plot(TimeVec,difference);
difference = abs(h-x_est_loop7(1,:));
plot(TimeVec,difference);
difference = abs(h-x_est_loop8(1,:));
plot(TimeVec,difference);
legend('x = [h;v;a;p;T;Tdot] Pressure not influencing height','x=[h,v,a,aoffset,p,T,Tdot] Pressure not influencing height','x=[h,v,a,aoffset,T]','x=[h,v,a,aoffset]','x=[h,v,a,aoffset] Without GPS','x=[h,v,a,aoffset,p] LowPass Pressure','x=[h,v,a,aoffset,p] a as input','x=[h,v,a,aoffset,phi]');
hold off;

figure('Name','Plot of the absolute height Errors System 1-8 sorted');
hold on;
grid on;
difference = abs(h-x_est_loop(1,:));
plot(TimeVec,sort(difference));
difference = abs(h-x_est_loop2(1,:));
plot(TimeVec,sort(difference));
difference = abs(h-x_est_loop3(1,:));
plot(TimeVec,sort(difference));
difference = abs(h-x_est_loop4(1,:));
plot(TimeVec,sort(difference));
difference = abs(h-x_est_loop5(1,:));
plot(TimeVec,sort(difference));
difference = abs(h-x_est_loop6(1,:));
plot(TimeVec,sort(difference));
difference = abs(h-x_est_loop7(1,:));
plot(TimeVec,sort(difference));
difference = abs(h-x_est_loop8(1,:));
plot(TimeVec,sort(difference));
legend('x = [h;v;a;p;T;Tdot] Pressure not influencing height','x=[h,v,a,aoffset,p,T,Tdot] Pressure not influencing height','x=[h,v,a,aoffset,T]','x=[h,v,a,aoffset]','x=[h,v,a,aoffset] Without GPS','x=[h,v,a,aoffset,p] LowPass Pressure','x=[h,v,a,aoffset,p] a as input','x=[h,v,a,aoffset,phi]');
hold off;
%%
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
% figure('Name','Pressure difference')
% for k = 1:length(TimeVec)
%     h6(k) = CalcHeight(x_est_loop6(5,1),x_est_loop6(5,k),T0,0,true);
% end
% h6est2 = x_est_loop6(1,:);
% 
% plot(abs(p-x_est_loop6(5,:)));
% hold on;
% plot(abs(p-p_mes_1));
% legend('with press as height','pressure lowpass filtered')
% median(abs(p-x_est_loop6(5,:)))
% median(abs(p-p_mes_1))