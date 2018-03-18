% In this script the State Estimation from given Sensor Data or self produced should be
% compared to the ground truth.

clear all; close all;

%% Initalize the System

% x1 = Height, x2 = Velocity, x3 Acceloration
A = [0 1 0;0 0 1; 0 0 0];
B = [0;0;0];                %No direct input
C = [1 0 0;0 0 1];          %Output ist Height and Acceleration
D = [0;0];
G = [0;0;1];                %System noise only on acceloration



% Get the time vector from the ARIS simulation.
load('TimeFromHassan.mat');
TimeVec = t';
Tau =  TimeVec(end)/length(TimeVec);


 %Diskretierung der Systemmatritzen
Ad = expm(A*Tau);     
Gd = Ad * G;
Bd = Ad * B;


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


%% Produce sensor Data self

figure('Name','Real Data');
% get acceloration by differentiate height:
a = diff(diff(h)/Tau)/Tau;
% Ad zeros to maintain vector length
a = [a 0 0];

% get height of GPS by deleting engouh values so it becomes 5Hz sample rate
GPSTau = 1/5;

%h_mes_GPS = zeros(1,length(h));
%h_me_GPS = decimate(h,uint(GPSTau/Tau));
for k =  1:(length(h)/round(GPSTau/Tau))
    for t =  1:round(GPSTau/Tau)
        h_mes_GPS((k-1)*round(GPSTau/Tau)+t) = h((k-1)*round(GPSTau/Tau)+1);
   end
end

% get Barometric Data (pressure)
% Pressure Data Temp/Po are just assumptions !!!!

g = 9.81;
m = 0.029; %Mass of average air
k = 1.3806503e-23;   %Bolzman const
R = 8.314472;    %Gas const
Po = 1013.25;    %Pressure at altitude 0
T = 15 + 273.15;    %Temperature in Kelvin
p1_mes = Po*exp(-(m*g*h)./(k*T));


plot(h,'b');
hold on;
plot(h_mes_GPS)
plot(a,'r');
plot(p1_mes);
legend('Real height in z','GPS heigt in z','Real Acceloration','Pressure');
hold off;

%% Add noise to sensor data

figure('Name','Noise Data');
plot(pnoise,'g');
plot(a,'r');
plot(anoise,'y');
legend('Position Real','Position Measured','Acceloration Real','Acceloration Measured');
hold off;


%% State estiamation loop

% NoiseMatices
R = [0 0;0 0];      %Sensor noise
Q = [0];            %System noise

% Initialize
u = zeros(1,length(TimeVec));  %Input vector is zero
y = [0;0];           %Output are the measurements
x = [0;0;0];                   %Is reality
P = [1 0 0;0 1 0;0 0 1];


% Loop
for k = 1:length(TimeVec)
    K = P*C'*pinv(C*P*C' + R);
    x = x + K*(y(:,k) - C*x);
    P = (eye(3)-K*C)*P;
    
    s(k)=x(1); v(k)=x(2); aes(k)=x(3);  %Save data from the Sensor fusion
    
    x = Ad*x + B*u(k);
    P = Ad*P*Ad' + Gd*Q*Gd';

end

%% Plot
