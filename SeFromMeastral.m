% Reads recorded sensor data
% - Convert sensors_pro_lowerbody.log into a Mat file
% 
% Logfile convention can be found at 
% https://github.com/DeinFreund/RpiRocket/blob/9f77e3df5d5fb0ff1a93765243e1763c402ef50d/accel.cpp
% 
% Required file: sensors_log_data.log (sensors_pro_lowerbody.log)
%       NOTE: please change the source manually in this script to read
%       other data
% 
% Other m-files required:   none
% MAT-files required: none


%------------- BEGIN CODE --------------


clear all; clc; close all; % remove that


log_time = [];     % Time of IMO sample: synch. with gyro/accel Nx[s]
log_imu_a = [];     % Accelerometer: Accels    ax, ay, az  Nx3*[m/s^2]
log_imu_g = [];     % Gyroscope, Angular rates gx, gy, gz  Nx3*[rad/s]
log_temp  = [];     % Temperature?
log_press = [];     % Pressure sensor: Mx3*[time[s], pressure[Pa/m^2], altitude[m]]
log_pos   = [];     % Nx[time  xpos     ypos     zpos        zpos2]
log_h     = [];     % Nx[time  press    alti     altispeed]
log_vel   = [];     % Nx[time  xvel     yvel     zvel        zvel2]
log_rot   = [];     % Nx[time  rot.v.x  rot.v.y  rot.v.z     rot.s]
log_b_sensors_noise = true; % true if sensors contain noise

% i=0;

% Open log file
fid = fopen('../SensorData/MaestralTest1/sensors_raw.log', 'r');
if fid == -1
   error('Cannot open file: %s', FileName);
end

% Skip first 2 lines
Line = fgetl(fid);
Line = fgetl(fid);

     isEOB = false;
     iData = 0;
     while ~isEOB && ~feof(fid)
%          i=i+1
        % Read new line
        Line = fgetl(fid);
        if ~ischar(Line) || strncmp(Line, 'EOB', 3)
           isEOB = true;
        else
            % Read line of logfile
            [data, num, err, ind1] = sscanf(Line, '%f %s %f %f %f', 10);
            sample_time = data(1); 
            if(sample_time > 100 & sample_time < 210)
            sample_id = char(data(2)); 
            
            % TODO: Data types checks, handle formatting errors
            if sample_id=='g'      % Gyroscope data
                log_time = [log_time; sample_time];
                log_imu_g = [log_imu_g; data(3), data(4), data(5)];
            elseif sample_id=='a'  % Accelerometer data (always after gyro)
                if not(sample_time==log_time(end))
                    error('IMU error: Gyro&Accel not synchronized?!');
                else
                    log_imu_a = [log_imu_a; data(3), data(4), data(5)];
                end
            elseif sample_id=='t'
                log_temp = [log_temp; data(1), data(3), data(4)];    
            elseif sample_id=='p' % can denote both position and pressure...
                if num==6 % Position   %  time     xpos    ypos   zpos  zpos2
                    log_pos = [log_pos; data(1), data(3), data(4), data(5), data(6)];
                elseif num == 4 % Temperature
                    log_press = [log_press; data(1), data(3), data(4)];
                else
                    error('Didn''manage to read ''p'' line of logfile');
                end
            elseif sample_id=='h' %  time     press    alti   altispeed
                log_h = [log_h; data(1), data(3), data(4), data(5)];  
            elseif sample_id=='v' %  time     xvel    yvel   zvel  zvel2
                log_vel = [log_vel; data(1), data(3), data(4), data(5), data(6)];  
            elseif sample_id=='r' %  time    rot.v.x  rot.v.y  rot.v.z  rot.s
                log_rot = [log_rot; data(1), data(3), data(4), data(5), data(6)];    
            end
           
            end
        end
     end
%   end

fclose(fid);

%% Do something with it

figure('Name','Pressure and Acceloration');
grid on;
for k = 1:length(log_temp)
    hnew(k) = CalcHeight(log_press(1,2),log_press(k,2),log_temp(1,2),0,false);
end
plot(log_press(:,1),hnew);
hold on;
plot(log_time,log_imu_a(:,2));

% Integrate the angle
angle = zeros(length(log_time),1);
%angle(1) = 0;
for k = 2:length(log_time)
    dT = log_time(k) - log_time(k-1);
    angle(k) = angle(k-1) + ((log_imu_g(k,3)-log_imu_g(k-1,3))/2)*dT;
end

%newAngle = filter(0.999,[1 1-0.999],log_imu_g(:,1));
%angle = cumsum(log_imu_g(:,3));
%a_mes = log_imu_a(:,3).*-cos(angle*pi/180);%log_imu_g(:,3)*pi/180);
az = log_imu_a(:,3).*-cos(angle*pi/180)*9.81;
plot(log_time,az);
plot(log_time,angle)
hold off;

%% Find Icongnition Time und Burnduration:
acc_mes = az;

T_ico_ind = 1; %Icogntiono Time index
while az(T_ico_ind) < 10
   T_ico_ind = T_ico_ind + 1; 
end

T_ico = log_time(T_ico_ind); %Icogniotion time

T_brn_ind = T_ico_ind; %Burnout time index
while az(T_brn_ind) > 10
    T_brn_ind = T_brn_ind +1;
end

T_brn = log_time(T_brn_ind); %Burnout time


T_par_ind = T_brn_ind; % Parachute time index

while abs(az(T_par_ind)) < 15
   T_par_ind = T_par_ind + 1; 
end

T_par = log_time(T_par_ind); % Parachute time

%% Calculate Mean, Variance and Bandwith before Icognition, during Burntime and during ubpflight until parachute activation from Accelerometer:
fs = 1/(log_time(end)/length(log_time));
figure('Name','Polyfit,Autocorrelation and power density spectrum before icognition Accel');

% Mean and Variance before icognition
decayTime = 3;
a_mean = mean(acc_mes(1:T_ico_ind-decayTime));
a_var_preIco  = var((acc_mes(1:T_ico_ind-decayTime)-a_mean));

subplot(4,1,1);
plot(log_time(1:T_ico_ind-decayTime),acc_mes(1:T_ico_ind-decayTime),log_time(1:T_ico_ind-decayTime),ones(1,length(log_time(1:T_ico_ind-decayTime)))*a_mean);

% Bandwith
a_noise_preIco = acc_mes(1:T_ico_ind-decayTime)-a_mean;
acorr_preIco = xcorr(a_noise_preIco);
N = length(acorr_preIco);

subplot(4,1,2);
plot([-N/2:N/2-1],acorr_preIco);
title('Autocorrelation');

subplot(4,1,3);
if length(acorr_preIco) < 2048
    N = 2048;
end
plot([-fs:(2*fs)/N:fs-2*fs/N],10*log10(fftshift(fft(abs(acorr_preIco),N))));
ylabel('[dB]'), xlabel('Frequency in [Hz]');
title('power density spectrum');

subplot(4,1,4);
histogram(acc_mes(1:T_ico_ind-decayTime)-a_mean);
title('Histogram');

% fing the best polynom to recreate curve during burntime:
figure('Name','Autocorrelation and Leistungsdichtespektrum between Icognition and Burnout');
riseTime = 9;
decayTime = 3;
p_brn = polyfit(log_time(T_ico_ind+riseTime:T_brn_ind-decayTime),acc_mes(T_ico_ind+riseTime:T_brn_ind-decayTime),2);
p_brn_curve = polyval(p_brn,log_time(T_ico_ind+riseTime:T_brn_ind-decayTime));
a_var_brn = var(acc_mes(T_ico_ind+riseTime:T_brn_ind-decayTime));

subplot(4,1,1);
plot(log_time(T_ico_ind+riseTime:T_brn_ind-decayTime),acc_mes(T_ico_ind+riseTime:T_brn_ind-decayTime),log_time(T_ico_ind+riseTime:T_brn_ind-decayTime),p_brn_curve);

% Bandwith
a_noise_brn = acc_mes(T_ico_ind+riseTime:T_brn_ind-decayTime)-p_brn_curve;
acorr_brn = xcorr(a_noise_brn);
N = length(acorr_brn);

subplot(4,1,2);
stem([-N/2:N/2-1],acorr_brn);
title('Autocorrelation');

subplot(4,1,3);
if length(acorr_brn) < 2048
    N = 2048;
end
stem([-fs:(2*fs)/N:fs-2*fs/N],10*log10(fftshift(fft(abs(acorr_brn),N))));
ylabel('Energy in [dB]'), xlabel('Frequency in [Hz]');
title('power density spectrum');

subplot(4,1,4);
histogram(acc_mes(T_ico_ind+riseTime:T_brn_ind-decayTime)-p_brn_curve);
title('Histogram');

% fing the best polynom to recreate curve during upflight:
figure('Name','Autocorrelation and Leistungsdichtespektrum after Burnout');
riseTime = 20;
decayTime = 12;
p_upflight = polyfit(log_time(T_brn_ind+riseTime:T_par_ind-decayTime),acc_mes(T_brn_ind+riseTime:T_par_ind-decayTime),2);
p_upflight_curve =polyval(p_upflight,log_time(T_brn_ind+riseTime:T_par_ind-decayTime));
a_var_upflight = var(acc_mes(T_brn_ind+riseTime:T_par_ind-decayTime)-p_upflight_curve);

subplot(4,1,1);
plot(log_time(T_brn_ind+riseTime:T_par_ind-decayTime),acc_mes(T_brn_ind+riseTime:T_par_ind-decayTime),log_time(T_brn_ind+riseTime:T_par_ind-decayTime),p_upflight_curve);

% Bandwith
a_noise_upflight = acc_mes(T_brn_ind+riseTime:T_par_ind-decayTime)-p_upflight_curve;
acorr_upflight = xcorr(a_noise_upflight);
N = length(acorr_upflight);
subplot(4,1,2);
stem([-N/2:N/2-1],acorr_upflight);
title('Autocorrelation');

subplot(4,1,3);
if length(acorr_upflight) < 2048
    N = 2048;
end
plot([-fs:(2*fs)/N:fs-2*fs/N],10*log10(fftshift(fft(abs(acorr_upflight),N))));
ylabel('Energy in [dB]'), xlabel('Frequency in [Hz]');
title('power density spectrum');

subplot(4,1,4);
histogram(acc_mes(T_brn_ind+riseTime:T_par_ind-decayTime)-p_upflight_curve);
title('Histogram');


%% Find the right indexes in the Barometer Timevector because this was sampled slower:

T_ico_ind = 1;
while log_press(T_ico_ind,1) < T_ico
    T_ico_ind =  T_ico_ind + 1;
end
T_brn_ind = T_ico_ind;
while log_press(T_brn_ind,1) < T_brn
    T_brn_ind = T_brn_ind + 1;
end
T_par_ind = T_brn_ind;
while log_press(T_par_ind,1) < T_par
    T_par_ind = T_par_ind + 1;
end


%% Same for Barometer pressure:

figure('Name','Polyfit,Autocorrelation and power density spectrum before icognition Pressure');

decayTime = 3;
p_mean = mean(log_press(1:T_ico_ind-decayTime,2));
p_var_preIco  = var(log_press(1:T_ico_ind-decayTime,2)-p_mean);


subplot(4,1,1);
plot(log_press(1:T_ico_ind-decayTime,1),log_press(1:T_ico_ind-decayTime,2),log_press(1:T_ico_ind-decayTime,1),ones(T_ico_ind-decayTime,1).*p_mean);
title('mean vs real');

% Bandwith
p_noise_preIco = log_press(1:T_ico_ind-decayTime,2)-p_mean;
pcorr_preIco = xcorr(p_noise_preIco);
N = length(pcorr_preIco);

subplot(4,1,2);
plot([-N/2:N/2-1],pcorr_preIco);
title('Autocorrelation');

subplot(4,1,3);
if length(pcorr_preIco) < 2048
    N = 2048;
end
plot([-fs:(2*fs)/N:fs-2*fs/N],10*log10(fftshift(fft(abs(pcorr_preIco),N))));
ylabel('[dB]'), xlabel('Frequency in [Hz]');
title('power density spectrum');

subplot(4,1,4);
histogram(log_press(1:T_ico_ind-decayTime,2)-p_mean);
title('Histogram');

figure('Name','Autocorrelation and power density spectrum between Icognition and Burnout Pressure');
%find the best polynom to recreate curve during burntime:
riseTime = 6;
decayTime = 3;
p_brn = polyfit(log_press(T_ico_ind+riseTime:T_brn_ind-decayTime,1),log_press(T_ico_ind+riseTime:T_brn_ind-decayTime,2),2);
p_brn_curve = polyval(p_brn,log_press(T_ico_ind+riseTime:T_brn_ind-decayTime,1));
p_var_brn = var(log_press(T_ico_ind+riseTime:T_brn_ind-decayTime,2)-p_brn_curve);

%For checking purpose:
subplot(4,1,1);
plot(log_press(T_ico_ind+riseTime:T_brn_ind-decayTime,1),log_press(T_ico_ind+riseTime:T_brn_ind-decayTime,2),log_press(T_ico_ind+riseTime:T_brn_ind-decayTime,1),p_brn_curve);
title('Poly vs real');

% Bandwith
p_noise_brn = log_press(T_ico_ind+riseTime:T_brn_ind-decayTime,2)-p_brn_curve;
pcorr_brn = xcorr(p_noise_brn);
N = length(pcorr_brn);

subplot(4,1,2);
plot([-N/2:N/2-1],pcorr_brn);
title('Autocorrelation');

subplot(4,1,3);
if length(pcorr_brn) < 2048
    N = 2048;
end
plot([-fs:(2*fs)/N:fs-2*fs/N],10*log10(fftshift(fft(abs(pcorr_brn),N))));
ylabel('[dB]'), xlabel('Frequency in [Hz]');
title('power density spectrum');

subplot(4,1,4);
histogram(p_noise_brn);
title('Histogram');

figure('Name','Autocorrelation and power density spectrum after Burnout Pressure');
%find the best polynom to recreate curve during upflight:
riseTime = 3;
decayTime = 20;
p_upflight = polyfit(log_press(T_brn_ind+riseTime:T_par_ind-decayTime,1),log_press(T_brn_ind+riseTime:T_par_ind-decayTime,2),2);
p_upflight_curve =polyval(p_upflight,log_press(T_brn_ind+riseTime:T_par_ind-decayTime,1));
p_var_upflight = var(log_press(T_brn_ind+riseTime:T_par_ind-decayTime,2)-p_upflight_curve);

%For checking purpose:
subplot(4,1,1);
plot(log_press(T_brn_ind+riseTime:T_par_ind-decayTime,1),log_press(T_brn_ind+riseTime:T_par_ind-decayTime,2),log_press(T_brn_ind+riseTime:T_par_ind-decayTime,1),p_upflight_curve);
title('Poly vs real');

% Bandwith
p_noise_upflight = log_press(T_brn_ind+riseTime:T_par_ind-decayTime,2)-p_upflight_curve;
pcorr_upflight = xcorr(p_noise_upflight);
N = length(pcorr_upflight);
subplot(4,1,2);
plot([-N/2:N/2-1],pcorr_upflight);
title('Autocorrelation');

subplot(4,1,3);
if length(pcorr_upflight) < 2048
    N = 2048;
end
plot([-fs:(2*fs)/N:fs-2*fs/N],10*log10(fftshift(fft(abs(pcorr_upflight),N))));
ylabel('[dB]'), xlabel('Frequency in [Hz]');
title('power density spectrum');

subplot(4,1,4);
histogram(p_noise_upflight);
title('Histogram');

%% Same for Barometer Temperature:

figure('Name','Polyfit,Autocorrelation and power density spectrum before icognition Temperatur');
decayTime = 3;
p_preIco = polyfit(log_press(1:T_ico_ind-decayTime,1),log_temp(1:T_ico_ind-decayTime,2),2);
p_preIco_curve = polyval(p_preIco,log_press(1:T_ico_ind-decayTime,1));
T_var_preIco = var(log_temp(1:T_ico_ind-decayTime,2)-p_preIco_curve);
%T_mean = sum(log_temp(1:T_ico_ind-decayTime,2))/(T_ico_ind-decayTime);
%T_var_preIco  = sum((log_temp(1:T_ico_ind-decayTime,2)-T_mean).^2)/(T_ico_ind-decayTime-1);
subplot(4,1,1);
plot(log_temp(1:T_ico_ind-decayTime,1),log_temp(1:T_ico_ind-decayTime,2),log_temp(1:T_ico_ind-decayTime,1),p_preIco_curve);%ones(T_ico_ind-decayTime,1)*T_mean);

% Bandwith
T_noise_preIco = log_temp(1:T_ico_ind-decayTime,2)-p_preIco_curve;
Tcorr_preIco = xcorr(T_noise_preIco);
N = length(Tcorr_preIco);

subplot(4,1,2);
plot([-N/2:N/2-1],Tcorr_preIco);
title('Autocorrelation');

subplot(4,1,3);
if length(Tcorr_preIco) < 2048
    N = 2048;
end
plot([-fs:(2*fs)/N:fs-2*fs/N],10*log10(fftshift(fft(abs(Tcorr_preIco),N))));
ylabel('[dB]'), xlabel('Frequency in [Hz]');
title('power density spectrum');

subplot(4,1,4);
histogram(T_noise_preIco);
title('Histogram');

figure('Name','Autocorrelation and power density spectrum between Icognition and Burnout Temperatur');

%find the best polynom to recreate curve during burntime:
riseTime = 6;
decayTime = 3;
p_brn = polyfit(log_press(T_ico_ind+riseTime:T_brn_ind-decayTime,1),log_temp(T_ico_ind+riseTime:T_brn_ind-decayTime,2),2);
p_brn_curve = polyval(p_brn,log_press(T_ico_ind+riseTime:T_brn_ind-decayTime,1));
T_var_brn = var(log_press(T_ico_ind+riseTime:T_brn_ind-decayTime,1)-p_brn_curve);

%For checking purpose:
subplot(4,1,1);
plot(log_press(T_ico_ind+riseTime:T_brn_ind-decayTime,1),log_temp(T_ico_ind+riseTime:T_brn_ind-decayTime,2),log_press(T_ico_ind+riseTime:T_brn_ind-decayTime,1),p_brn_curve);

% Bandwith
T_noise_brn = log_temp(T_ico_ind+riseTime:T_brn_ind-decayTime,2)-p_brn_curve;
Tcorr_brn = xcorr(T_noise_brn);
N = length(Tcorr_brn);

subplot(4,1,2);
plot([-N/2:N/2-1],Tcorr_brn);
title('Autocorrelation');

subplot(4,1,3);
if length(Tcorr_brn) < 2048
    N = 2048;
end
plot([-fs:(2*fs)/N:fs-2*fs/N],10*log10(fftshift(fft(abs(Tcorr_brn),N))));
ylabel('[dB]'), xlabel('Frequency in [Hz]');
title('power density spectrum');

subplot(4,1,4);
histogram(T_noise_brn);
title('Histogram');

figure('Name','Autocorrelation and power density spectrum after Burnout Temperatur');

%find the best polynom to recreate curve during upflight:
riseTime = 3;
decayTime = 2;
p_upflight = polyfit(log_press(T_brn_ind+riseTime:T_par_ind-decayTime,1),log_temp(T_brn_ind+riseTime:T_par_ind-decayTime,2),2);
p_upflight_curve =polyval(p_upflight,log_press(T_brn_ind+riseTime:T_par_ind-decayTime,1));
T_var_upflight = var(log_temp(T_brn_ind+riseTime:T_par_ind-decayTime,2)-p_upflight_curve);

%For checking purpose:
subplot(4,1,1);
plot(log_press(T_brn_ind+riseTime:T_par_ind-decayTime,1),log_temp(T_brn_ind+riseTime:T_par_ind-decayTime,2),log_press(T_brn_ind+riseTime:T_par_ind-decayTime,1),p_upflight_curve);

% Bandwith
T_noise_upflight = log_temp(T_brn_ind+riseTime:T_par_ind-decayTime,2)-p_upflight_curve;
Tcorr_upflight = xcorr(T_noise_upflight);
N = length(Tcorr_upflight);

subplot(4,1,2);
plot([-N/2:N/2-1],Tcorr_upflight);
title('Autocorrelation');

subplot(4,1,3);
if length(Tcorr_upflight) < 2048
    N = 2048;
end
plot([-fs:(2*fs)/N:fs-2*fs/N],10*log10(fftshift(fft(abs(Tcorr_upflight),N))));
ylabel('[dB]'), xlabel('Frequency in [Hz]');
title('power density spectrum');

subplot(4,1,4);
histogram(T_noise_upflight);
title('Histogram');

%% State Estimation:

%System

A = [0 1 0 0 0; 0 0 1 0 0; 0 0 0 0 0;0 0 0 0 0;-0.00649 0 0 0 0];
B = [0;0;0;0;0];
C = [1 0 0 0 0; 0 0 1 -1 0;0 0 0 0 1];                                     % Input is Height out of Pressure, Acceloration and TemP
D = [0];
G = [0 0 0;0 0 0;1 0 0;0 1 0;0 0 1];

%Discretizise

TimeVec = log_time';
TimeVecPress = log_press(:,1)';
Tau = (TimeVec(end)-TimeVec(1))/length(TimeVec);
Ad = expm(A*Tau);     

Bd = Ad * B;
Gd = Ad * G
disp(['Size of A: ' num2str(size(A))]);

%% Observability test;
disp(['Rank: ' num2str(rank([C;C*Ad;C*Ad*Ad]))]);

%% Get to the neeeded Matrices:

%Calculate noise of sensors out of the Roro Data befor icgnognitiona.
Tico = T_ico-0.002;                 %Icoginition time point
Burntime = T_brn-T_ico; % [s]
%Sensor Noise:
k = 1;
while TimeVec(k) < Tico
    k= k+1;
end
MACL = (1/(length(az(1:k))-1)*sum((az(1:k)-0).^2))/50    
k = 1;
while TimeVecPress(k) < Tico
    k =k+1;
end
MPRS = (1/(length(hnew(1:k))-1)*sum((hnew(1:k)-0).^2))%/50    

T0 = sum(log_temp(1:k,2))/k;
MTMP = (1/(length(log_temp(1:k,2))-1)*sum((log_temp(1:k,2)-T0).^2))

R = diag([2^32;MACL;2^32]);


%Static System noise:
HGT = 0;
SPE = 0;
ACEL = 0.10;
ACELOFS = 0.00005;
TMP = 0.005;
%Q = diag([HGT;SPE;ACEL;ACELOFS;TMP]);
Q = diag([ACEL;ACELOFS;TMP]);

% %Dynamic Sytem noise:
% HGT = ones(1,length(TimeVec))*HGT;
% SPE = ones(1,length(TimeVec))*SPE;
% ACEL = ones(1,length(TimeVec))*ACEL;
% TMP = ones(1,length(TimeVec))*TMP;
% Q_dyn = [HGT;SPE;ACEL;TMP];  
% %Add all noise vectors into an noise matrix
% Q_dyn_m = diag(Q_dyn(:,1)');
% for n = 2:length(TimeVec)
%     Q_dyn_m = cat(3,Q_dyn_m,diag(Q_dyn(:,n)'));
% end


%% Initialize
u = zeros(1,length(TimeVec));                       %Input vector is zero
%y = [P_mes;AccWGyro_mes;Temp_mes];                 %Output are the measurements
%y_t = timeseries(y,TimeVec);
x0 = [0;0;0;0;log_temp(1,2)];                         %Start points
x =  x0;                                            %Is reality
Po = 1013.25;
P = eye(5);
StaticEst = false;%true;

%% Loop

x_est_loop = zeros(size(x,1),length(TimeVec));      %Vector for the SE values
Height = CalcHeight(log_press(1,2),log_press(1,2),log_temp(1,2),0,false);
Temp = log_temp(1,2);

lengthp = 0;
disp('Start estimation loop...');

for k = 1:length(TimeVec)
    %Calculate Tau an adjust A Matrix
    if k > 1
        Tau = TimeVec(k)-TimeVec(k-1);
        Ad = expm(A*Tau);
        Gd = Ad * G;
    end
    
        x = Ad*x;% + Bd*u(k);
    
    if StaticEst
        P = Ad*P*Ad' + Gd*Q*Gd'; 
    else
        if TimeVec(k) > Tico & TimeVec(k) < Tico + Burntime
            %P = Ad*P*Ad' + Gd'*diag([HGT;SPE;ACEL;0;TMP])*Gd; 
            P = Ad*P*Ad' + Gd*diag([ACEL*100000;ACELOFS;TMP])*Gd'; 
        else
            P = Ad*P*Ad' + Gd*Q*Gd'; 
        end
        
    end
    
    %Determine if it is between after Icognition;
    if TimeVec(k) > Tico & TimeVec(k) < Tico + Burntime
        TempMACL = MACL;%* 200;
    else
        TempMACL = MACL;
    end
    %Determine if a Press/Temp measurement is also avaiables
    index = find((TimeVec(k)-0.0003)< TimeVecPress & TimeVecPress < (TimeVec(k) + 0.0003));
    if index ~= 0;
        lengthp = lengthp+1;
        TempMPRS = MPRS;
        TempMTMP = MTMP;
        Height = CalcHeight(log_press(1,2),log_press(index(1),2),log_temp(1,2),0,false);
        Temp = log_temp(index(1),2);
    else
        TempMPRS = 2^320;
        TempMTMP = 2^320;
    end
    K = P*C'*pinv(C*P*C' + diag([TempMPRS;TempMACL;TempMTMP]));
    
    x = x + K*([Height;acc_mes(k);Temp] - C*x);
    P = (eye(5)-K*C)*P;
    
    x_est_loop(:,k) = x;                            %Save data from the Sensor fusion
    

end

disp('...finished !!');

%%  Plot the estimated values
figure('Name','Estimated Values')
hold on;
grid on;
plot(TimeVec,x_est_loop(1,:));
plot(TimeVecPress,hnew);
plot(TimeVec,x_est_loop(2,:));
plot(TimeVec,x_est_loop(3,:));
plot(log_time,az);
plot(TimeVec,x_est_loop(4,:));
plot(TimeVec,x_est_loop(5,:));
plot(TimeVecPress,log_temp(:,2));
legend('Height','Height out of Pressure','Speed Estimated','Acceloration Estimated','Acceloration Measured','Acceloration Offset','Temperatur','Temperature Measured');
xlabel('Time [s]');
