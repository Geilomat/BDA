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


log_imu_t = [];     % Time of IMO sample: synch. with gyro/accel Nx[s]
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
            if(sample_time > 190 & sample_time < 270)
            sample_id = char(data(2)); 
            
            % TODO: Data types checks, handle formatting errors
            if sample_id=='g'      % Gyroscope data
                log_imu_t = [log_imu_t; sample_time];
                log_imu_g = [log_imu_g; data(3), data(4), data(5)];
            elseif sample_id=='a'  % Accelerometer data (always after gyro)
                if not(sample_time==log_imu_t(end))
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

for k = 1:length(log_temp)
    hnew(k) = CalcHeight(log_press(1,2),log_press(k,2),log_temp(k,2),0,false);
end
plot(log_press(:,1),hnew);