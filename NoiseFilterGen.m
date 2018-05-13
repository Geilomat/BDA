%% Script to Generate the filter and check them
% Needed data: mean, var, corr of the data eg: a_mean, a_var_treIco,
% acorr_preIco

close all, clear all, clc;

%% Get the different AR model Parameters with the walker yule equation of order n


n = 100;                %Order of the AR models
dTMeasured = 0.013;     %Sample Timesteps from the test flights
dTSimulation = 0.001;   %Sample Timesteps for the Simulation
% generated the different noise vectors:
% noisePreIco = p_noise_preIco;
% noiseBrn = _brn;
% noiseUpflight = p_noise_upflighht;
sensor = 'a'        % which sensor should be calculated, a,p,T,phi,GPS
saveData = false;   % If the AR-Models should be saved

 if sensor == 'a'
    load('Ericnoise.mat');
    noisePreIco = a_noise_preIco;
    TVPreIco = a_T_preIco-a_T_preIco(1);
    noiseBrn = a_noise_brn;
    TVBrn = a_T_brn-a_T_brn(1);
    noiseUpflight = a_noise_upflight;
    TVUpflight = a_T_upflight-a_T_upflight(1);
    load('Tir1noise.mat');
    noisePreIco = [noisePreIco;a_noise_preIco];
    TVPreIco = [TVPreIco;a_T_preIco-a_T_preIco(1)+TVPreIco(end)+dT];
    noiseBrn = [noiseBrn; a_noise_brn];
    TVBrn = [TVBrn;a_T_brn-a_T_brn(1)+TVBrn(end)+dT];
    noiseUpflight = [noiseUpflight; a_noise_upflight];
    TVUpflight = [TVUpflight; a_T_upflight-a_T_upflight(1)+TVUpflight(end)+dT];
    load('Tir2noise.mat');
    noisePreIco = [noisePreIco;a_noise_preIco];
    TVPreIco = [TVPreIco;a_T_preIco-a_T_preIco(1)+TVPreIco(end)+dT];
    noiseBrn = [noiseBrn; a_noise_brn];
    TVBrn = [TVBrn;a_T_brn-a_T_brn(1)+TVBrn(end)+dT];
    noiseUpflight = [noiseUpflight; a_noise_upflight];
    TVUpflight = [TVUpflight; a_T_upflight-a_T_upflight(1)+TVUpflight(end)+dT];
    load('Gregnoise.mat');
    noisePreIco = [noisePreIco;a_noise_preIco];
    TVPreIco = [TVPreIco;a_T_preIco-a_T_preIco(1)+TVPreIco(end)+dT];
    noiseBrn = [noiseBrn; a_noise_brn];
    TVBrn = [TVBrn;a_T_brn-a_T_brn(1)+TVBrn(end)+dT];
    noiseUpflight = [noiseUpflight; a_noise_upflight];
    TVUpflight = [TVUpflight; a_T_upflight-a_T_upflight(1)+TVUpflight(end)+dT];
    fname = 'h_a.mat' % in which mat file the system coefficent should be stored
 end
 
 
 if sensor == 'p'
    load('Ericnoise.mat');
    noisePreIco = p_noise_preIco;
    TVPreIco = p_T_preIco-p_T_preIco(1);
    noiseBrn = p_noise_brn;
    TVBrn = p_T_brn-p_T_brn(1);
    noiseUpflight = p_noise_upflight;
    TVUpflight = p_T_upflight-p_T_upflight(1);
    load('Tir1noise.mat');
    noisePreIco = [noisePreIco;p_noise_preIco];
    TVPreIco = [TVPreIco;p_T_preIco-p_T_preIco(1)+TVPreIco(end)+dT];
    noiseBrn = [noiseBrn; p_noise_brn];
    TVBrn = [TVBrn;p_T_brn-p_T_brn(1)+TVBrn(end)+dT];
    noiseUpflight = [noiseUpflight; p_noise_upflight];
    TVUpflight = [TVUpflight; p_T_upflight-p_T_upflight(1)+TVUpflight(end)+dT];
    load('Tir2noise.mat');
    noisePreIco = [noisePreIco;p_noise_preIco];
    TVPreIco = [TVPreIco;p_T_preIco-p_T_preIco(1)+TVPreIco(end)+dT];
    noiseBrn = [noiseBrn; p_noise_brn];
    TVBrn = [TVBrn;p_T_brn-p_T_brn(1)+TVBrn(end)+dT];
    noiseUpflight = [noiseUpflight; p_noise_upflight];
    TVUpflight = [TVUpflight; p_T_upflight-p_T_upflight(1)+TVUpflight(end)+dT];
    load('Gregnoise.mat');
    noisePreIco = [noisePreIco;p_noise_preIco];
    TVPreIco = [TVPreIco;p_T_preIco-p_T_preIco(1)+TVPreIco(end)+dT];
    noiseBrn = [noiseBrn; p_noise_brn];
    TVBrn = [TVBrn;p_T_brn-p_T_brn(1)+TVBrn(end)+dT];
    noiseUpflight = [noiseUpflight; p_noise_upflight];
    TVUpflight = [TVUpflight; p_T_upflight-p_T_upflight(1)+TVUpflight(end)+dT];
    fname = 'h_p.mat' % in which mat file the system coefficent should be stored
 end


 if sensor == 'T'
    load('Ericnoise.mat');
    noisePreIco = T_noise_preIco;
    TVPreIco = T_T_preIco-T_T_preIco(1);
    noiseBrn = T_noise_brn;
    TVBrn = T_T_brn-T_T_brn(1);
    noiseUpflight = T_noise_upflight;
    TVUpflight = T_T_upflight-T_T_upflight(1);
    load('Tir1noise.mat');
    noisePreIco = [noisePreIco;T_noise_preIco];
    TVPreIco = [TVPreIco;T_T_preIco-T_T_preIco(1)+TVPreIco(end)+dT];
    noiseBrn = [noiseBrn; T_noise_brn];
    TVBrn = [TVBrn;T_T_brn-T_T_brn(1)+TVBrn(end)+dT];
    noiseUpflight = [noiseUpflight; T_noise_upflight];
    TVUpflight = [TVUpflight; T_T_upflight-T_T_upflight(1)+TVUpflight(end)+dT];
    load('Tir2noise.mat');
    noisePreIco = [noisePreIco;T_noise_preIco];
    TVPreIco = [TVPreIco;T_T_preIco-T_T_preIco(1)+TVPreIco(end)+dT];
    noiseBrn = [noiseBrn; T_noise_brn];
    TVBrn = [TVBrn;T_T_brn-T_T_brn(1)+TVBrn(end)+dT];
    noiseUpflight = [noiseUpflight; T_noise_upflight];
    TVUpflight = [TVUpflight; T_T_upflight-T_T_upflight(1)+TVUpflight(end)+dT];
    load('Gregnoise.mat');
    noisePreIco = [noisePreIco;T_noise_preIco];
    TVPreIco = [TVPreIco;T_T_preIco-T_T_preIco(1)+TVPreIco(end)+dT];
    noiseBrn = [noiseBrn; T_noise_brn];
    TVBrn = [TVBrn;T_T_brn-T_T_brn(1)+TVBrn(end)+dT];
    noiseUpflight = [noiseUpflight; T_noise_upflight];
    TVUpflight = [TVUpflight; T_T_upflight-T_T_upflight(1)+TVUpflight(end)+dT];
    fname = 'h_T.mat' % in which mat file the system coefficent should be stored
 end
 
 
 if sensor == 'phi'
    noisePreIco = [];
    noiseBrn = [];
    noiseUpflight = [];
    TVPreIco = [];
    TVBrn = [];
    TVUpflight = [];
    disp('not yet implemented');
    fname = 'h_phi.mat' % in which mat file the system coefficent should be stored
 end
 
 
 
 if sensor == 'GPS'
    noiseLength = 100;
    noiseFrequency = 0.5;
    GPSVar = 0.8;
    noisePreIco = randn(noiseLength,1)*sqrt(GPSVar);
    noiseBrn = randn(noiseLength,1)*sqrt(GPSVar);
    noiseUpflight = randn(noiseLength,1)*sqrt(GPSVar);
    TVPreIco = [0:1/noiseFrequency:1/noiseFrequency*noiseLength-1]';
    TVBrn = [0:1/noiseFrequency:1/noiseFrequency*noiseLength-1]';
    TVUpflight = [0:1/noiseFrequency:1/noiseFrequency*noiseLength-1]';
    %disp('not yet implemented');
    fname = 'h_GPS.mat' % in which mat file the system coefficent should be stored
 end
 
%  noisePreIco = a_noise_preIco;
%  TVPreIco = a_T_preIco;
%  noiseBrn = a_noise_brn;
%  TVBrn = a_T_brn-a_T_brn(1);
%  noiseUpflight = a_noise_upflight;
%  TVUpflight = a_T_upflight-a_T_upflight(1);
 %% Adjust the Noise vector to the right time sampling
fs = 1/dTSimulation;

noisePreIco = resample(noisePreIco,TVPreIco,fs);
noiseBrn = resample(noiseBrn,TVBrn,fs);
noiseUpflight = resample(noiseUpflight,TVUpflight,fs);


%% Calculate AR-Models

[h_preIco estVar_preIco] = aryule(noisePreIco,n);
[h_brn estVar_brn] = aryule(noiseBrn,n);
[h_upflight estVar_upflight] = aryule(noiseUpflight,n);

%% Define those Variables to test the estimatet AR models from above

varPreIco = estVar_preIco;

varBrn = estVar_brn;

varUpflight = estVar_upflight;


%% Compare to the measured before icognition

h = h_preIco;
mes_noise = noisePreIco;
mes_var = varPreIco;

gen_noise = randn(length(mes_noise),1) *sqrt(mes_var);%+a_mean;
test_noise = filter(1,h,gen_noise);
test_corr = xcorr(test_noise);
mes_corr = xcorr(mes_noise);
figure('Name','Autocorrelation during upflight');
plot(mes_corr);
hold on;
plot(test_corr);
legend('acorr real','acorr test');

figure('Name','Power density during upflight');
plot(10*log10(pwelch(mes_noise)));
%pwelch(noise,test_noise);
hold on;
%pwelch(test_noise);
plot(10*log10(pwelch(test_noise)));
legend('PDS measured','PDS test')


%% Compare while burning

h = h_brn;
mes_noise = noiseBrn;
mes_var = varBrn;

gen_noise = randn(length(mes_noise),1) *sqrt(mes_var);%+a_mean;
test_noise = filter(1,h,gen_noise);
test_corr = xcorr(test_noise);
mes_corr = xcorr(mes_noise);
figure('Name','Autocorrelation during upflight');
plot(mes_corr);
hold on;
plot(test_corr);
legend('acorr real','acorr test');

figure('Name','Power density during upflight');
plot(10*log10(pwelch(mes_noise)));
%pwelch(noise,test_noise);
hold on;
%pwelch(test_noise);
plot(10*log10(pwelch(test_noise)));
legend('PDS measured','PDS test')


%% Compare after burnout

h = h_upflight;
mes_noise = noiseUpflight;
mes_var = varUpflight;

gen_noise = randn(length(mes_noise),1) *sqrt(mes_var);%+a_mean;
test_noise = filter(1,h,gen_noise);
test_corr = xcorr(test_noise);
mes_corr = xcorr(mes_noise);
figure('Name','Autocorrelation during upflight');
plot(mes_corr);
hold on;
plot(test_corr);
legend('acorr real','acorr test');

figure('Name','Power density during upflight');
plot(10*log10(pwelch(mes_noise)));
%pwelch(noise,test_noise);
hold on;
%pwelch(test_noise);
plot(10*log10(pwelch(test_noise)));
legend('PDS measured','PDS test')



%% Save them into the right file:

if saveData == true
    save(fname,'h_preIco','h_brn','h_upflight','varPreIco','varBrn','varUpflight','dTMeasured');
    disp(['saved AR-Models into: ' fname]);
end