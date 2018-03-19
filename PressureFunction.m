clear all; close all;
clc;
%% Implement the different vetors
NmbT = 5;
for k = 1:NmbT
    T0(k) = (k-1)*15 + 273.15;   %K°
end


P0 = 1013.25;       %hPa on 0 altitude
h = 0:5:3000;

for k = 1:length(T0)
    T(k,:) = T0(k)-0.00649*h;
end
T(k+1,:) = ones(1,length(h))*288.15;

%% Get the different pressure curves
for k = 1:length(T0)
    p(k,:) = P0*(1-(0.0065*h)./T(k,:)).^5.255;
end
p(length(T0)+1,:) = P0*(1-(0.0065*h)./288.15).^5.255;
%% Plot to See Changes
figure('Name','Pressure depending on start Temp and height')
hold on;
grid on;
for k = 1:NmbT
    plot3(p(k,:),h,T(k,:));
end
plot3(p(k+1,:),h,T(k+1,:));
legend('0','15','30','45','60','StaticTemp 15C°');
xlabel('Pressure [hPa]');
ylabel('Height [m]');
zlabel('Temp [k°]');

%% Static Vs. Dynamic

p_Stac(1,:) = P0*(1-(0.0065*h)./288.15).^5.255;
p_Stac(2,:) = P0*(1-(0.0065*h)./T(2,:)).^5.255;

figure('Name','Pressure with static and changing Temp');
hold on;
plot(h,p_Stac(1,:));
plot(h,p_Stac(2,:));
ylabel('Pressure [hPa]');
xlabel('Height [m]');
legend('Static','Dynamic');

%% Try to regenerate this function in a linear fashion by T0 = 15°:

deltaPH = p_Stac(1,end)-p_Stac(1,1);
deltaT = T(2,end)-T(2,1);
deltaH = h(end)-h(1);
deltaPT = p_Stac(2,end)-p_Stac(1,end);
TempGain = deltaPT/deltaT;
HeightGain = deltaPH/deltaH;

plin15 = P0 + TempGain * (T(2,:)-T0(2)) + HeightGain * h;

figure('Name','Linearised function VS. real func');
grid on;
hold on;
plot(p_Stac(1,:),h);
plot(p_Stac(2,:),h);
xlabel('Pressure [hPa]');
ylabel('Height [m]');
plot(plin15,h);
legend('Static','Dynamic','Linearized');

%% Same as above with 60 Degrees
p_Stac(1,:) = P0*(1-(0.0065*h)./(273.15+60)).^5.255;
p_Stac(2,:) = P0*(1-(0.0065*h)./T(5,:)).^5.255;

deltaPH = p_Stac(1,end)-p_Stac(1,1);
deltaT = T(5,end)-T(5,1);
deltaH = h(end)-h(1);
deltaPT = p_Stac(2,end)-p_Stac(1,end);
TempGain = deltaPT/deltaT;
HeightGain = deltaPH/deltaH;

plin60 = P0 + TempGain * (T(5,:)-T0(5)) + HeightGain * h;

figure('Name','Linearised function VS. real func');
grid on;
hold on;
plot(p_Stac(1,:),h);
plot(p_Stac(2,:),h);
xlabel('Pressure [hPa]');
ylabel('Height [m]');
plot(plin60,h);
legend('Static','Dynamic','Linearized');

%% Create lookuptable for different Starting Temperatures (0-100 C°):

for k = 1:50
    T0(k) = (k-1)*2 + 273.15;   %K°
end

P0 = 1013.25;       %hPa on 0 altitude
h = 0:5:3000;

for k = 1:length(T0)
    T(k,:) = T0(k)-0.00649*h;
end

for k = 1:50

p_Stac(1,:) = P0*(1-(0.0065*h)./(273.15+(k-1)*2)).^5.255;
p_Stac(2,:) = P0*(1-(0.0065*h)./T(k,:)).^5.255;

deltaPH = p_Stac(1,end)-p_Stac(1,1);
deltaT = T(5,end)-T(5,1);
deltaH = h(end)-h(1);
deltaPT = p_Stac(2,end)-p_Stac(1,end);
PressLookUp(1,k) = deltaPT/deltaT; %First column Temperature Gain
PressLookUp(2,k) = deltaPH/deltaH; %Secon column Height Gain  
end

save('PressLookUp.mat','PressLookUp');