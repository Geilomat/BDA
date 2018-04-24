%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Modul MDS Laborversuch 1
% Wasserrakete
% 
%
% Modulverantwortlicher: Dr. Christoph Eck
% Assistent: Benedikt Imbach
% 05.04.2018

%
% Mit der dazugehörigen Datei WaterRocket.mdl wird eine Wasserrakete
% bestehend aus einer Pet-Flasche gefüllt mit einem unter Druck stehenden
% Wasser-Luftgemisch simuliert. 
% Bei T0 wird die Flaschenöffnung freigegeben.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clc;
clear;

pa = 0.97e5;        % ambient pressure ,Pa
p0 = 4e5;           % pressure in Rocket at T = 0 ,Pa
rho = 1000;         % density water ,kg/m^3
F = 0.01^2*pi;      % area nozzle m^2
A = (0.095/2)^2*pi; % area Rocket m^2
Vl = 1.58e-3;       % volume Rocket m^3
ml = 0.113;         % mass empty Rocket kg
g = 9.81;           % earth gravitational acceleration m/s^2
kappa = 1.35;       % kappa of air -
airdensity = 1.204; % air density (20°C, 1013 mBar) ,kg/m^2
cw = 0.4;           % cw value of a bottle

mw0 = 0.53*Vl*rho;   % mass water
V0 = Vl-mw0/rho;    % volume air T0

%initial conditions
v0 = 0.0;           % initial velocity
h0 = 0.3;           % initial height


% Optimierte Wassermenge
ymax = 0;
v_opt = 0;

for v = 0.3:0.01:0.8
    mw0 = v*Vl*rho;   % mass water
    V0 = Vl-mw0/rho;    % volume air T0
    sim('WaterRocket_uebung.mdl');
    if max(yout(:,(3))) >= ymax;
        ymax = max(yout(:,3));
        v_opt = v;
    end
    figure(1)
    hold on
    plot(tout,yout(:,(3)));
    title('Brute-Force Verfahren')
    xlabel('Zeit [s]')
    ylabel('Höhe [m]')
    pause(0.1)
    shg
end


mw0 = v_opt*Vl*rho;   % mass water
    V0 = Vl-mw0/rho;    % volume air T0
% simulink feedback
% return value is:
%   tout => time vector
%   yout => acc, vel, hight, pressure[bar], total mass,  thurst
sim('WaterRocket_uebung.mdl');

figure(2)
plot(tout,yout);   
title('Optimierte Raketen Simulation')
xlabel('Zeit [s]')
legend('accel.','velocity','hight','pressure','total mass','thrust');
shg

mw0*1000/rho

