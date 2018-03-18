close all; clear all;
clc;

%% Kalman filter for a Projectil with chanching acceloration

% x1 = Height, x2 = Velocity, x3 Acceloration
g = 9.81;
A = [0 1 0;0 0 1; 0 0 0];
B = [0;0;0];        %No direct inpout
C = [1 0 0;0 0 1];        %Output ist Height and Acceleration
D = [0;0];
G = [0;0;1];       %Noise matrix

T = 0.01;           %Measurement every 10 ms;


Ad = expm(A*T)
Gd = Ad * G

%% Beobachtbarkeit:

rank([C;C*Ad;C*Ad*Ad])
%p = 950*(1./(1+exp(-0.006*950.*Time1)*(950./1-1)))

%%  Flugbahn:
%Time = 0:0.001:30;
load('StateFromHassan.mat');
load('TimeFromHassan.mat');

p = state(:,3);%[sin(Time)];
p =  p';
Time = t';
T =  Time(length(Time))/length(Time);
%p = [2200*(1./(1+exp(-0.000999*2200.*Time1)*(2200./1-1))) 3000*sin(Time2/(20/pi)) 3000*sin(Time3/(20/pi))];
%a = [zeros(1,200),ones(1,100)*10*g,zeros(1,100),ones(1,500)*-g];
%p = cumtrapz(cumtrapz(a)*T)*T;
a = [diff(diff(p)/T)/T 0 0];%[zeros(1,200) -sin(Time)]; %p dot dot
%flatting out the position.
% p(500) = (p(499)+p(501))/2;
% p(501) = (p(500)+p(502))/2;
% p(502) = (p(501)+p(503))/2;
% p(503) = (p(502)+p(504))/2;
% p(504) = (p(503)+p(505))/2;
%a = imu_a(:,3)'; %%Calculating Acceloration => p_dotdot.
% %Flatting the acceloration out.
% a(503) = (a(499)+a(504))/2;
% a(502) = (a(499)+a(503))/2;
% a(501) = (a(499)+a(502))/2;
% a(500) = (a(499)+a(501))/2;


figure(1)
plot(p,'b') %%Original bahn
hold on;


pnoise = p;awgn(p,0.01,'measured');
anoise = a;awgn(a,0.01,'measured');
plot(pnoise,'g');
plot(a,'r');
plot(anoise,'y');
legend('Position Real','Position Measured','Acceloration Real','Acceloration Measured');
hold off;
%% Kalman Loop;
 Q = (0.001/3)^2;   % Varianz Systemrauschen;
 varp = (1/(length(pnoise)-1)*sum((pnoise-p).^2)) %Calculate Vairance of position
 vara = (1/(length(anoise)-1)*sum((anoise-a).^2)) %Calculate Variance of accelaration
 R = [varp 0;0 vara];     % Varianz Messrauschen;
 
 u = zeros(1,length(pnoise));   %Input vector is zero
 y = [pnoise;anoise];           %Output are the measurements
 %initalitazion
 x = [0;0;0];                   %Is reality
 P = [1 0 0;0 1 0;0 0 1];
 
 %Simualtion
 for k =1:length(p)

    K = P*C'*pinv(C*P*C' + R);
    x = x + K*(y(:,k) - C*x);
    P = (eye(3)-K*C)*P;
    
    s(k)=x(1); v(k)=x(2); aes(k)=x(3);  %Save data from the Sensor fusion
    
    x = Ad*x + B*u(k);
    P = Ad*P*Ad' + Gd*Q*Gd';
     
     
 end
 
 figure(2)
 plot(s,'r');
 hold on;
 plot(p,'g');
 plot(aes,'b');
 plot(a,'y');
 legend('Position Estimate','Position Real','Acceloration Estimate','Acceloration Real');
 hold off;


