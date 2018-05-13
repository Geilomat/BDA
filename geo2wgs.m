function [x, y, z] = geo2wgs(lat, lon, alt)

a = 6378137.0;               % WGS-84 semi-major axis
e2 = 6.6943799901377997e-3;  % WGS-84 first eccentricity squared
a1 = 4.2697672707157535e+4;  % a1 = a*e2
a2 = 1.8230912546075455e+9;  % a2 = a1*a1
a3 = 1.4291722289812413e+2;  % a3 = a1*e2/2
a4 = 4.5577281365188637e+9;  % a4 = 2.5*a2
a5 = 4.2840589930055659e+4;  % a5 = a1+a3
a6 = 9.9330562000986220e-1;  % a6 = 1-e2

n = a./sqrt(1-e2.*sin(lat).*sin(lat));
x = (n+alt).*cos(lat).*cos(lon);
y = (n+alt).*cos(lat).*sin(lon);
z = (n.*(1-e2)+alt).*sin(lat);

end