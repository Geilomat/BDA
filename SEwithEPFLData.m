clear all; clc; close all; % remove that
% Sensordata
% seqNmbr time altitude airspeed pressure temp ax ay az gx gy gz gx gy gz
% GPS data
% empty time number of sats hdop lat lon altitude
hasGPS = false;

log_time = [];     % Time of IMO sample: synch. with gyro/accel Nx[s]
log_imu_a = [];     % Accelerometer: Accels    ax, ay, az  Nx3*[g]
log_imu_g = [];     % Gyroscope, Angular rates gx, gy, gz  Nx3*[°/s]
log_temp  = [];     % Temperature?
log_press = [];     % Pressure sensor:[time[s], pressure[hPa], airspeed]
log_h     = [];     % Nx[time altituded]
log_airspeed = [];  % [airspeed]

% Open log file uncomment for different Flights.
%load('Tir_test3d2_Paths.mat');
%load('Tir_test3d1_Paths.mat');
%load('18_11_18_Eric_Paths.mat');
load('18_11_18_Greg_Paths.mat');

if hasGPS
    log_GPS_time = [];
    log_GPS_pos = [];
    log_GPS_sat = [];
end

for k = 1:size(Paths);
    
fid = fopen(Paths(k,:), 'r');
if fid == -1
   error('Cannot open file: %s', FileName);
end

% Skip first 2 lines
% Line = fgetl(fid);
% Line = fgetl(fid);

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
            [data, num, err, ind1] = sscanf(Line, '%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f', 15);
            %sample_time = data(2); 
            %if(sample_time > 185 & sample_time < 300)
            %sample_id = char(data(2)); 
            log_time = [log_time; data(2)];
            %log_data = [log_data; data(3:end)'];
            log_h = [log_h;data(3)];
            log_airspeed = [log_airspeed; data(4)];
            log_press = [log_press; data(5)];
            log_temp = [log_temp; data(6)];
            log_imu_a = [log_imu_a; data(7) data(8) data(9)];
            log_imu_g = [log_imu_g; data(10) data(11) data(12)];     
            end
     end
%   end

fclose(fid);
end

if hasGPS
for k = 1:size(PathsGPS);
    
fid = fopen(PathsGPS(k,:), 'r');
if fid == -1
   error('Cannot open file: %s', FileName);
end

     isEOB = false;
     iData = 0;
     while ~isEOB && ~feof(fid)
        % Read new line
        Line = fgetl(fid);
        if ~ischar(Line) || strncmp(Line, 'EOB', 3)
           isEOB = true;
        else
            % Read line of logfile
            [data, num, err, ind1] = sscanf(Line, '%f %f %f %f %f %f', 6);
            %sample_time = data(2); 
            %if(sample_time > 185 & sample_time < 300)
            log_GPS_time = [log_GPS_time; data(1)];
            log_GPS_sat = [log_GPS_sat;data(2)];
            log_GPS_pos = [log_GPS_pos; data(3) data(4) data(5) data(6)];     
            end
     end

fclose(fid);
end
startTime = log_GPS_time(1);
log_GPS_time(:) = log_GPS_time(:)-startTime;
log_GPS_time(:) = log_GPS_time(:)/1000000;
end

% Adjust time vector;
startTime = log_time(1);
log_time(:) = log_time(:)-startTime;
log_time(:) = log_time(:)/1000000;


%%
plot(log_time,log_h);
hold on;
%plot(log_time,log_press);
%plot(log_GPS_time, log_GPS_pos(:,4)+22);

%% Testing

%plot(log_time,log_imu_g)
plot(log_time,log_imu_a);
legend('1','2','3');
hold on;
%hold on; 
%plot(log_time,log_h)


%figure('Name','Accel without/with gyro')
angle = zeros(length(log_time),1);
%angle(1) = 0;
for k = 2:length(log_time)
    dT = log_time(k) - log_time(k-1);
    angle(k) = angle(k-1) + (log_imu_g(k,3) -log_imu_g(k-1,3))*dT;
end

az = log_imu_a(:,2) .* -cos(angle*pi/180)*9.81;

%plot(log_time,log_imu_a(:,1)*9.81);
%hold on;
plot(log_time,az);
%plot(log_time,log_h);
%legend('without gyro','with gyro','height');

%% Find Icongnition Time und Burnduration:

T_ico_ind = 1; %Icogntiono Time index
while az(T_ico_ind) < 20
   T_ico_ind = T_ico_ind + 1; 
end

T_ico = log_time(T_ico_ind); %Icogniotion time

T_brn_ind = T_ico_ind; %Burnout time index
while az(T_brn_ind) > 10
    T_brn_ind = T_brn_ind +1;
end

T_brn = log_time(T_brn_ind); %Burnout time


T_par_ind = T_brn_ind; % Parachute time index

while abs(az(T_par_ind)) < 10.1
   T_par_ind = T_par_ind + 1; 
end

T_par = log_time(T_par_ind); % Parachute time



%% Calculate Mean an Variance before Icognition, during Burntime and during ubpflight until parachute activation from Accelerometer:

decayTime = 3;
a_mean = sum(az(1:T_ico_ind-decayTime))/(T_ico_ind-decayTime);
a_var_preIco  = sum((az(1:T_ico_ind-decayTime)-a_mean).^2)/(T_ico_ind-decayTime-1);

% fing the best polynom to recreate curve during burntime:
riseTime = 9;
decayTime = 3;
p_brn = polyfit(log_time(T_ico_ind+riseTime:T_brn_ind-decayTime),az(T_ico_ind+riseTime:T_brn_ind-decayTime),2);
p_brn_curve = polyval(p_brn,log_time(T_ico_ind+riseTime:T_brn_ind-decayTime));
a_var_brn = sum((az(T_ico_ind+riseTime:T_brn_ind-decayTime)-p_brn_curve).^2)/(T_brn_ind-decayTime-(T_ico_ind+riseTime)-1);

%For checking purpose:
plot(log_time(T_ico_ind+riseTime:T_brn_ind-decayTime),az(T_ico_ind+riseTime:T_brn_ind-decayTime),log_time(T_ico_ind+riseTime:T_brn_ind-decayTime),p_brn_curve);

% fing the best polynom to recreate curve during upflight:
riseTime = 20;
decayTime = 12;
p_upflight = polyfit(log_time(T_brn_ind+riseTime:T_par_ind-decayTime),az(T_brn_ind+riseTime:T_par_ind-decayTime),2);
p_upflight_curve =polyval(p_upflight,log_time(T_brn_ind+riseTime:T_par_ind-decayTime));
a_var_upflight = sum((az(T_brn_ind+riseTime:T_par_ind-decayTime)-p_upflight_curve).^2)/(T_par_ind-decayTime-(T_brn_ind+riseTime)-1);

%For checking purpose:
plot(log_time(T_brn_ind+riseTime:T_par_ind-decayTime),az(T_brn_ind+riseTime:T_par_ind-decayTime),log_time(T_brn_ind+riseTime:T_par_ind-decayTime),p_upflight_curve);

%% Same for Barometer pressure:

decayTime = 3;
p_mean = sum(log_press(1:T_ico_ind-decayTime))/(T_ico_ind-decayTime);
p_var_preIco  = sum((log_press(1:T_ico_ind-decayTime)-p_mean).^2)/(T_ico_ind-decayTime-1);

%find the best polynom to recreate curve during burntime:
riseTime = 6;
decayTime = 3;
p_brn = polyfit(log_time(T_ico_ind+riseTime:T_brn_ind-decayTime),log_press(T_ico_ind+riseTime:T_brn_ind-decayTime),2);
p_brn_curve = polyval(p_brn,log_time(T_ico_ind+riseTime:T_brn_ind-decayTime));
p_var_brn = sum((log_press(T_ico_ind+riseTime:T_brn_ind-decayTime)-p_brn_curve).^2)/(T_brn_ind-decayTime-(T_ico_ind+riseTime)-1);

%For checking purpose:
plot(log_time(T_ico_ind+riseTime:T_brn_ind-decayTime),log_press(T_ico_ind+riseTime:T_brn_ind-decayTime),log_time(T_ico_ind+riseTime:T_brn_ind-decayTime),p_brn_curve);

%find the best polynom to recreate curve during upflight:
riseTime = 3;
decayTime = 20;
p_upflight = polyfit(log_time(T_brn_ind+riseTime:T_par_ind-decayTime),log_press(T_brn_ind+riseTime:T_par_ind-decayTime),2);
p_upflight_curve =polyval(p_upflight,log_time(T_brn_ind+riseTime:T_par_ind-decayTime));
p_var_upflight = sum((log_press(T_brn_ind+riseTime:T_par_ind-decayTime)-p_upflight_curve).^2)/(T_par_ind-decayTime-(T_brn_ind+riseTime)-1);

%For checking purpose:
plot(log_time(T_brn_ind+riseTime:T_par_ind-decayTime),log_press(T_brn_ind+riseTime:T_par_ind-decayTime),log_time(T_brn_ind+riseTime:T_par_ind-decayTime),p_upflight_curve);


%% Same for Barometer Temperature:

decayTime = 3;
T_mean = sum(log_temp(1:T_ico_ind-decayTime))/(T_ico_ind-decayTime);
T_var_preIco  = sum((log_temp(1:T_ico_ind-decayTime)-T_mean).^2)/(T_ico_ind-decayTime-1);

%find the best polynom to recreate curve during burntime:
riseTime = 6;
decayTime = 3;
p_brn = polyfit(log_time(T_ico_ind+riseTime:T_brn_ind-decayTime),log_temp(T_ico_ind+riseTime:T_brn_ind-decayTime),2);
p_brn_curve = polyval(p_brn,log_time(T_ico_ind+riseTime:T_brn_ind-decayTime));
T_var_brn = sum((log_temp(T_ico_ind+riseTime:T_brn_ind-decayTime)-p_brn_curve).^2)/(T_brn_ind-decayTime-(T_ico_ind+riseTime)-1);

%For checking purpose:
plot(log_time(T_ico_ind+riseTime:T_brn_ind-decayTime),log_temp(T_ico_ind+riseTime:T_brn_ind-decayTime),log_time(T_ico_ind+riseTime:T_brn_ind-decayTime),p_brn_curve);

%find the best polynom to recreate curve during upflight:
riseTime = 3;
decayTime = 2;
p_upflight = polyfit(log_time(T_brn_ind+riseTime:T_par_ind-decayTime),log_temp(T_brn_ind+riseTime:T_par_ind-decayTime),2);
p_upflight_curve =polyval(p_upflight,log_time(T_brn_ind+riseTime:T_par_ind-decayTime));
T_var_upflight = sum((log_temp(T_brn_ind+riseTime:T_par_ind-decayTime)-p_upflight_curve).^2)/(T_par_ind-decayTime-(T_brn_ind+riseTime)-1);

%For checking purpose:
plot(log_time(T_brn_ind+riseTime:T_par_ind-decayTime),log_temp(T_brn_ind+riseTime:T_par_ind-decayTime),log_time(T_brn_ind+riseTime:T_par_ind-decayTime),p_upflight_curve);

%% Path generation
%clear all;
%hasGPS = true;
%PathsGPS(1,:) = '../SensorData/EPFLData/24.02.2018.1 - Tir_test3D/gps_data_004_1519469099934';
%PathsGPS(2,:) = '../SensorData/EPFLData/24.02.2018.2-Tir_test3D_fix/gps_data_001_1519479278350';
% PathsGPS(3,:) = '../SensorData/EPFLData/18.11.2017.2 - Greg/telemetry_data_3_1511012347617';
% PathsGPS(4,:) = '../SensorData/EPFLData/18.11.2017.2 - Greg/telemetry_data_4_1511012369307';
% PathsGPS(5,:) = '../SensorData/EPFLData/18.11.2017.2 - Greg/telemetry_data_5_1511012401268';

