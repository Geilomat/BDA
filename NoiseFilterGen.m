%% Script to Generate the filter and check them
% Needed data: mean, var, corr of the data eg: a_mean, a_var_treIco,
% acorr_preIco

close all, clear all, clc;

%% Get the different AR model Parameters with the walker yule equation of order n


n = 20;

% generated the different noise vectors:
% noisePreIco = p_noise_preIco;
% noiseBrn = p_noise_brn;
% noiseUpflight = p_noise_upflighht;
sensor = 'a' % which sensor should be calculated, a,p,T,phi,GPS
fname = 'h_a.mat' % in which mat file the system coefficent should be stored

 if sensor == 'a'
    load('Ericnoise.mat');
    noisePreIco = a_noise_preIco;
    noiseBrn = a_noise_brn;
    noiseUpflight = a_noise_upflight;
    load('Tir1noise.mat');
    noisePreIco = [noisePreIco;a_noise_preIco];
    noiseBrn = [noiseBrn; a_noise_brn];
    noiseUpflight = [noiseUpflight; a_noise_upflight];
    load('Tir2noise.mat');
    noisePreIco = [noisePreIco;a_noise_preIco];
    noiseBrn = [noiseBrn; a_noise_brn];
    noiseUpflight = [noiseUpflight; a_noise_upflight];
    load('Gregnoise.mat');
    noisePreIco = [noisePreIco;a_noise_preIco];
    noiseBrn = [noiseBrn; a_noise_brn];
    noiseUpflight = [noiseUpflight; a_noise_upflight];
 end
 
 
 if sensor == 'p'
    load('Ericnoise.mat');
    noisePreIco = p_noise_preIco;
    noiseBrn = p_noise_brn;
    noiseUpflight = p_noise_upflight;
    load('Tir1noise.mat');
    noisePreIco = [noisePreIco;p_noise_preIco];
    noiseBrn = [noiseBrn; p_noise_brn];
    noiseUpflight = [noiseUpflight; p_noise_upflight];
    load('Tir2noise.mat');
    noisePreIco = [noisePreIco;p_noise_preIco];
    noiseBrn = [noiseBrn; p_noise_brn];
    noiseUpflight = [noiseUpflight; p_noise_upflight];
    load('Gregnoise.mat');
    noisePreIco = [noisePreIco;p_noise_preIco];
    noiseBrn = [noiseBrn; p_noise_brn];
    noiseUpflight = [noiseUpflight; p_noise_upflight];
 end


 if sensor == 'T'
    load('Ericnoise.mat');
    noisePreIco = T_noise_preIco;
    noiseBrn = T_noise_brn;
    noiseUpflight = T_noise_upflight;
    load('Tir1noise.mat');
    noisePreIco = [noisePreIco;T_noise_preIco];
    noiseBrn = [noiseBrn; T_noise_brn];
    noiseUpflight = [noiseUpflight; T_noise_upflight];
    load('Tir2noise.mat');
    noisePreIco = [noisePreIco;T_noise_preIco];
    noiseBrn = [noiseBrn; T_noise_brn];
    noiseUpflight = [noiseUpflight; T_noise_upflight];
    load('Gregnoise.mat');
    noisePreIco = [noisePreIco;T_noise_preIco];
    noiseBrn = [noiseBrn; T_noise_brn];
    noiseUpflight = [noiseUpflight; T_noise_upflight];
 end
 
 
 if sensor == 'phi'
    disp('not yet implemented');
 end
 
 
 
 if sensor == 'GPS'
    disp('not yet implemented');
 end


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

gen_noise = randn(length(mes_noise),1)*sqrt(mes_var);
test_noise = filter(1,h,gen_noise);
mes_corr = xcorr(mes_noise);
test_corr = xcorr(test_noise);
figure('Name','Autocorrelation before Icognition');
plot(mes_corr);
hold on;
plot(test_corr);
legend('acorr real','acorr test');

figure('Name','Power density before Icognition');
plot(10*log10(pwelch(mes_noise)));
hold on;
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
figure('Name','Autocorrelation while burning');
plot(mes_corr);
hold on;
plot(test_corr);
legend('acorr real','acorr test');

figure('Name','Power density while burning');
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

save(fname,'h_preIco','h_brn','h_upflight','varPreIco','varBrn','varUpflight');