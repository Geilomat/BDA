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
            if(sample_time > 180 & sample_time < 250.75)
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

figure('Name','Pressure and Acceloration');
grid on;
for k = 1:length(log_temp)
    hnew(k) = CalcHeight(log_press(1,2),log_press(k,2),log_temp(k,2),0,false);
end
plot(log_press(:,1),hnew);
hold on;
plot(log_imu_t,log_imu_a(:,3));

newAngle = filter(0.3,[1 1-0.3],log_imu_g(:,2));
a_mes = log_imu_a(:,3).*sin(newAngle*pi/180);%log_imu_g(:,3)*pi/180);
plot(log_imu_t,a_mes);
hold off;

%% State Estimation:

%System

A = [0 1 0 0; 0 0 1 0; 0 0 0 0;-0.00649 0 0 0];
B = [0;0;0;0];
C = [1 0 0 0; 0 0 1 0; 0 0 0 1];                                     % Input is Height out of Pressure, Acceloration and TemP
D = [0];

%Discretizise

TimeVec = log_imu_t';
TimeVecPress = log_press(:,1)';
Tau = (TimeVec(end)-TimeVec(1))/length(TimeVec);
Ad = expm(A*Tau);     

Bd = Ad * B;

disp(['Size of A: ' num2str(size(A))]);

%% Observability test;
disp(['Rank: ' num2str(rank([C;C*Ad;C*Ad*Ad]))]);

%% Get to the neeeded Matrices:

%Calculate noise of sensors out of the Roro Data befor icgnognitiona.
Tico = 199.2;                 %Icoginition time point

%Sensor Noise:
k = 1;
while TimeVec(k) < 199.2
    k= k+1;
end
MACL = (1/(length(a_mes(1:k))-1)*sum((a_mes(1:k)-0).^2))    
k = 1;
while TimeVecPress(k) < 199.2
    k =k+1;
end
MPRS = (1/(length(hnew(1:k))-1)*sum((hnew(1:k)-0).^2))    
T0 = sum(log_temp(1:k,2))/k;

MTMP = (1/(length(log_temp(1:k,2))-1)*sum((log_temp(1:k,2)-T0).^2))
R = diag([2^32;MACL;2^32]);

%Static System noise:
HGT = 0;
SPE = 0;
ACEL = 0.05;
TMP = 0.001;
Q = diag([HGT;SPE;ACEL;TMP]);

%Dynamic Sytem noise:
HGT = ones(1,length(TimeVec))*HGT;
SPE = ones(1,length(TimeVec))*SPE;
ACEL = ones(1,length(TimeVec))*ACEL;%[100 100 100 50 30 ones(1,length(TimeVec)-5)*ACEL];
TMP = ones(1,length(TimeVec))*TMP;
Q_dyn = [HGT;SPE;ACEL;TMP];  
%Add all noise vectors into an noise matrix
Q_dyn_m = diag(Q_dyn(:,1)');
for n = 2:length(TimeVec)
    Q_dyn_m = cat(3,Q_dyn_m,diag(Q_dyn(:,n)'));
end


%% Initialize
u = zeros(1,length(TimeVec));                       %Input vector is zero
%y = [P_mes;AccWGyro_mes;Temp_mes];                 %Output are the measurements
%y_t = timeseries(y,TimeVec);
x0 = [0;0;0;log_temp(1,2)];                         %Start points
x =  x0;                                            %Is reality
Po = 1032.15;
P = eye(4);
StaticEst = false;

%% Loop

x_est_loop = zeros(size(x,1),length(TimeVec));      %Vector for the SE values
Height = CalcHeight(log_press(1,2),log_press(1,2),log_temp(1,2),0,false);
Temp = log_temp(1,2);


disp('Start estimation loop...');

for k = 1:length(TimeVec)
    %Calculate Tau an adjust A Matrix
    if k > 1
        Tau = TimeVec(k)-TimeVec(k-1);
        Ad = expm(A*Tau);     
    end
    
    %Determine if it is between after Icognition;
    if TimeVec(k) > Tico & TimeVec(k) < Tico + 1.50
        TempMACL = MACL * 200;
    else
        TempMACL = MACL;
    end
    %Determine if a Press/Temp measurement is also avaiables
    index = find((TimeVec(k)-0.0005)< TimeVecPress & TimeVecPress < (TimeVec(k) + 0.0005));
    if index ~= 0;
        %if x(1) > 400           
            %K = P*C'*pinv(C*P*C' + [MPRS 0 0;0 MACL 0;0 0 0.01]);
        %else
         %   K = P*C'*pinv(C*P*C' + [0.05 0 0;0 MACL 0;0 0 0.01]);
        %end
        TempMPRS = MPRS;
        TempMTMP = MTMP;
        Height = CalcHeight(log_press(1,2),log_press(index(1),2),log_temp(index(1),2),0,false);
        Temp = log_temp(index(1),2);
    else
        TempMPRS = 2^32;
        TempMTMP = 2^32;
        %K = P*C'*pinv(C*P*C' + R);
    end
    K = P*C'*pinv(C*P*C' + diag([TempMPRS;TempMACL;TempMTMP]));
    
    x = x + K*([Height;a_mes(k);Temp] - C*x);
    P = (eye(4)-K*C)*P;
    
    x_est_loop(:,k) = x;                            %Save data from the Sensor fusion
    
    x = Ad*x;% + Bd*u(k);
    
    if StaticEst
        P = Ad*P*Ad' + Q; 
    else
        P = Ad*P*Ad' + Q_dyn_m(:,:,k); 
    end
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
plot(log_imu_t,a_mes);
plot(TimeVec,x_est_loop(4,:));
plot(TimeVecPress,log_temp(:,2));
legend('Height','Height out of Pressure','Speed','Acceloration','Acceloration Measured','Temperatur','Temperature Measured');
xlabel('Time [s]');
