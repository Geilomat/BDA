clear all; clc; close all; % remove that


log_time = [];     % Time of IMO sample: synch. with gyro/accel Nx[s]
log_data = []; 
log_imu_a = [];     % Accelerometer: Accels    ax, ay, az  Nx3*[m/s^2]
log_imu_g = [];     % Gyroscope, Angular rates gx, gy, gz  Nx3*[rad/s]
log_temp  = [];     % Temperature?
log_press = [];     % Pressure sensor:[time[s], pressure[Pa/m^2], airspeed]
% log_pos   = [];     % Nx[time  xpos     ypos     zpos        zpos2]
log_h     = [];     % Nx[time altituded]
log_airspeed = [];  % [airspeed]
log_gps   = [];
% log_vel   = [];     % Nx[time  xvel     yvel     zvel        zvel2]
% log_rot   = [];     % Nx[time  rot.v.x  rot.v.y  rot.v.z     rot.s]
% log_b_sensors_noise = true; % true if sensors contain noise

% i=0;

% Open log file uncomment for different Flights.
% EPFL Tir_test3D_fix;
%load('Tir_test3d2_Paths.mat');
%load('Tir_test3d1_Paths.mat');
%load('18_11_18_Eric_Paths.mat');
load('18_11_18_Greg_Paths.mat');


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
            
%             % TODO: Data types checks, handle formatting errors
%             if sample_id=='g'      % Gyroscope data
%                 log_imu_t = [log_imu_t; sample_time];
%                 log_imu_g = [log_imu_g; data(3), data(4), data(5)];
%             elseif sample_id=='a'  % Accelerometer data (always after gyro)
%                 if not(sample_time==log_imu_t(end))
%                     error('IMU error: Gyro&Accel not synchronized?!');
%                 else
%                     log_imu_a = [log_imu_a; data(3), data(4), data(5)];
%                 end
%             elseif sample_id=='t'
%                 log_temp = [log_temp; data(1), data(3), data(4)];    
%             elseif sample_id=='p' % can denote both position and pressure...
%                 if num==6 % Position   %  time     xpos    ypos   zpos  zpos2
%                     log_pos = [log_pos; data(1), data(3), data(4), data(5), data(6)];
%                 elseif num == 4 % Temperature
%                     log_press = [log_press; data(1), data(3), data(4)];
%                 else
%                     error('Didn''manage to read ''p'' line of logfile');
%                 end
%             elseif sample_id=='h' %  time     press    alti   altispeed
%                 log_h = [log_h; data(1), data(3), data(4), data(5)];  
%             elseif sample_id=='v' %  time     xvel    yvel   zvel  zvel2
%                 log_vel = [log_vel; data(1), data(3), data(4), data(5), data(6)];  
%             elseif sample_id=='r' %  time    rot.v.x  rot.v.y  rot.v.z  rot.s
%                 log_rot = [log_rot; data(1), data(3), data(4), data(5), data(6)];    
%             end
           
            end
     end
%   end

fclose(fid);
end

%% Adjust time vector;
startTime = log_time(1);
log_time(:) = log_time(:)-startTime;
log_time(:) = log_time(:)/1000000;
%%
clear all;
Paths(1,:) = '../SensorData/EPFLData/18.11.2017.2 - Greg/telemetry_data_1_1511012308145';
Paths(2,:) = '../SensorData/EPFLData/18.11.2017.2 - Greg/telemetry_data_2_1511012328160';
Paths(3,:) = '../SensorData/EPFLData/18.11.2017.2 - Greg/telemetry_data_3_1511012347617';
Paths(4,:) = '../SensorData/EPFLData/18.11.2017.2 - Greg/telemetry_data_4_1511012369307';
Paths(5,:) = '../SensorData/EPFLData/18.11.2017.2 - Greg/telemetry_data_5_1511012401268';
%Paths(6,:) = '../SensorData/EPFLData/18.11.2017.1 - Eric/telemetry/flight data/telemetry_data_6_1511008823759';
%Paths(7,:) = '../SensorData/EPFLData/18.11.2017.1 - Eric/telemetry/flight data/telemetry_data_7_1511008838265';
%Paths(8,:) = '../SensorData/EPFLData/18.11.2017.1 - Eric/telemetry/flight data/telemetry_data_8_1511008909814';
%Paths(9,:) = '../SensorData/EPFLData/18.11.2017.1 - Eric/telemetry/flight data/telemetry_data_1_1511008752975';
%Paths(6,:) = '../SensorData/EPFLData/24.02.2018.1 - Tir_test3D/sensors_data_005_1519479278352';
