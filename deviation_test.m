time = deviationtestkriensbalconycloudy.second+deviationtestkriensbalconycloudy.minute*60+deviationtestkriensbalconycloudy.hour*60*60;
time = time - time(1);
lat = deviationtestkriensbalconycloudy.Latitudedeg+deviationtestkriensbalconycloudy.Latitudemin/60;
lon = deviationtestkriensbalconycloudy.Longitudedeg+deviationtestkriensbalconycloudy.Longitudemin/60;
alt = deviationtestkriensbalconycloudy.Altitudemamsl;

[x, y, z] = geo2wgs(lat,lon,alt);

%%

alt_noise = alt - mean(alt);
% alt_sort = sort(abs(alt_noise));
% 
% index95 = round(length(alt_sort)*0.95);
% alt_sort(index95)

%%
alt_noise = resample(alt_noise,time,1000);
n = 100
%%
[h_flight varFlight] = aryule(alt_noise,n);

nois_new = filter(1,h_flight,randn(1,length(alt_noise))*sqrt(varFlight));

plot(10*log10(pwelch(nois_new)));
hold on;
plot(10*log10(pwelch(alt_noise)));
hold off;
% 
% figure(2)
% histogram(alt_noise)
save('h_GPS.mat','h_flight','varFlight');