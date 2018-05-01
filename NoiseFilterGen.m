%% Script to Generate the filter and check them
% Needed data: mean, var, corr of the data eg: a_mean, a_var_treIco,
% acorr_preIco

%% Get the different AR model Parameters with the walker yule equation of order n

n = 10;
noisePreIco = a_noise_preIco;
noiseBrn = a_noise_brn;
noiseUpflight = a_noise_upflight;

[h_preIco estVar_preIco] = aryule(noisePreIco,n);
[h_brn estVar_brn] = aryule(noiseBrn,n);
[h_upflight estVar_upflight] = aryule(noiseUpflight,n);

%% Define those Variables to test the estimatet FIR systems from above

varPreIco = estVar_preIco;
corrPreIco = acorr_preIco;

varBrn = estVar_brn;
corrBrn = acorr_brn;

varUpflight = estVar_upflight;
corrUpflight = acorr_upflight;

%% Compare to the measured before icognition

h = h_preIco;
mes_noise = noisePreIco;
mes_var = varPreIco;
mes_corr =  corrPreIco;

gen_noise = randn(length(mes_noise),1)*sqrt(mes_var);
test_noise = filter(1,h,gen_noise);
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
mes_corr =  corrBrn;

gen_noise = randn(length(mes_noise),1)*sqrt(mes_var);%+a_mean;
test_noise = filter(1,h,gen_noise);
test_corr = xcorr(test_noise);
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
mes_corr =  corrUpflight;

gen_noise = randn(length(mes_noise),1)*sqrt(mes_var);%+a_mean;
test_noise = filter(1,h,gen_noise);
test_corr = xcorr(test_noise);
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