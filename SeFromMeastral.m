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
            if(sample_time > 185 & sample_time < 300)
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
    angle(k) = angle(k-1) + (log_imu_g(k,3))*dT;
end

%newAngle = filter(0.999,[1 1-0.999],log_imu_g(:,1));
%angle = cumsum(log_imu_g(:,3));
%a_mes = log_imu_a(:,3).*-cos(angle*pi/180);%log_imu_g(:,3)*pi/180);
az = log_imu_a(:,3).*-cos(angle*pi/180);%*9.81;
plot(log_time,az);
hold off;

%% Find Icongnition Time und Burnduration:

T_ico_ind = 1; %Icogntiono Time index
while az(T_ico_ind) < 2
   T_ico_ind = T_ico_ind + 1; 
end

T_ico = log_time(T_ico_ind); %Icogniotion time

T_brn_ind = T_ico_ind; %Burnout time index
while az(T_brn_ind) > 1
    T_brn_ind = T_brn_ind +1;
end

T_brn = log_time(T_brn_ind); %Burnout time


T_par_ind = T_brn_ind; % Parachute time index

while abs(az(T_par_ind)) < 1.1
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
%plot(log_time(T_brn_ind+riseTime:T_par_ind-decayTime),az(T_brn_ind+riseTime:T_par_ind-decayTime),log_time(T_brn_ind+riseTime:T_par_ind-decayTime),p_upflight_curve);

%% Same for Barometer pressure:

decayTime = 3;
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

p_mean = sum(log_press(1:T_ico_ind-decayTime,2))/(T_ico_ind-decayTime);
p_var_preIco  = sum((log_press(1:T_ico_ind-decayTime,2)-p_mean).^2)/(T_ico_ind-decayTime-1);

%find the best polynom to recreate curve during burntime:
riseTime = 6;
decayTime = 3;
p_brn = polyfit(log_press(T_ico_ind+riseTime:T_brn_ind-decayTime,1),log_press(T_ico_ind+riseTime:T_brn_ind-decayTime,2),2);
p_brn_curve = polyval(p_brn,log_press(T_ico_ind+riseTime:T_brn_ind-decayTime,1));
p_var_brn = sum((log_press(T_ico_ind+riseTime:T_brn_ind-decayTime,2)-p_brn_curve).^2)/(T_brn_ind-decayTime-(T_ico_ind+riseTime)-1);

%For checking purpose:
plot(log_press(T_ico_ind+riseTime:T_brn_ind-decayTime,1),log_press(T_ico_ind+riseTime:T_brn_ind-decayTime,2),log_press(T_ico_ind+riseTime:T_brn_ind-decayTime,1),p_brn_curve);

%find the best polynom to recreate curve during upflight:
riseTime = 3;
decayTime = 20;
p_upflight = polyfit(log_press(T_brn_ind+riseTime:T_par_ind-decayTime,1),log_press(T_brn_ind+riseTime:T_par_ind-decayTime,2),2);
p_upflight_curve =polyval(p_upflight,log_press(T_brn_ind+riseTime:T_par_ind-decayTime,1));
p_var_upflight = sum((log_press(T_brn_ind+riseTime:T_par_ind-decayTime,2)-p_upflight_curve).^2)/(T_par_ind-decayTime-(T_brn_ind+riseTime)-1);

%For checking purpose:
%plot(log_press(T_brn_ind+riseTime:T_par_ind-decayTime,1),log_press(T_brn_ind+riseTime:T_par_ind-decayTime,2),log_press(T_brn_ind+riseTime:T_par_ind-decayTime,1),p_upflight_curve);


%% Same for Barometer Temperature:

decayTime = 3;
T_mean = sum(log_temp(1:T_ico_ind-decayTime,2))/(T_ico_ind-decayTime);
T_var_preIco  = sum((log_temp(1:T_ico_ind-decayTime,2)-T_mean).^2)/(T_ico_ind-decayTime-1);

%find the best polynom to recreate curve during burntime:
riseTime = 6;
decayTime = 3;
p_brn = polyfit(log_press(T_ico_ind+riseTime:T_brn_ind-decayTime,1),log_temp(T_ico_ind+riseTime:T_brn_ind-decayTime,2),2);
p_brn_curve = polyval(p_brn,log_press(T_ico_ind+riseTime:T_brn_ind-decayTime,1));
T_var_brn = sum((log_press(T_ico_ind+riseTime:T_brn_ind-decayTime,1)-p_brn_curve).^2)/(T_brn_ind-decayTime-(T_ico_ind+riseTime)-1);

%For checking purpose:
plot(log_press(T_ico_ind+riseTime:T_brn_ind-decayTime,1),log_temp(T_ico_ind+riseTime:T_brn_ind-decayTime,2),log_press(T_ico_ind+riseTime:T_brn_ind-decayTime,1),p_brn_curve);

%find the best polynom to recreate curve during upflight:
riseTime = 3;
decayTime = 2;
p_upflight = polyfit(log_press(T_brn_ind+riseTime:T_par_ind-decayTime,1),log_temp(T_brn_ind+riseTime:T_par_ind-decayTime,2),2);
p_upflight_curve =polyval(p_upflight,log_press(T_brn_ind+riseTime:T_par_ind-decayTime,1));
T_var_upflight = sum((log_temp(T_brn_ind+riseTime:T_par_ind-decayTime,2)-p_upflight_curve).^2)/(T_par_ind-decayTime-(T_brn_ind+riseTime)-1);

%For checking purpose:
%plot(log_press(T_brn_ind+riseTime:T_par_ind-decayTime,1),log_temp(T_brn_ind+riseTime:T_par_ind-decayTime,2),log_press(T_brn_ind+riseTime:T_par_ind-decayTime,1),p_upflight_curve);


%% State Estimation:

%System

A = [0 1 0 0 0; 0 0 1 0 0; 0 0 0 0 0;0 0 0 0 0;-0.00649 0 0 0 0];
B = [0;0;0;0;0];
C = [1 0 0 0 0; 0 0 1/9.81 -1/9.81 0;0 0 0 0 1];                                     % Input is Height out of Pressure, Acceloration and TemP
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
Tico = 197;                 %Icoginition time point
Burntime = 2.5; % [s]
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
ACEL = 0.001;
ACELOFS = 0.001;
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
            P = Ad*P*Ad' + Gd*diag([ACEL*1000;ACELOFS/10;TMP])*Gd'; 
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
    
    x = x + K*([Height;az(k);Temp] - C*x);
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
plot(log_time,az*9.81);
plot(TimeVec,x_est_loop(4,:));
plot(TimeVec,x_est_loop(5,:));
plot(TimeVecPress,log_temp(:,2));
legend('Height','Height out of Pressure','Speed Estimated','Acceloration Estimated','Acceloration Measured','Acceloration Offset','Temperatur','Temperature Measured');
xlabel('Time [s]');
