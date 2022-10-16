% Variable electromagnet gain
% Load data into variables
load('data_maglev.mat');
% 
tmin = 10; % min time for data analysis
tmax = 50; % max time for data analysis 
Ts = 1e-3; % sampling time of controller
imin = tmin/Ts+1;
imax = tmax / Ts; % maximum time to view plot (e.g. 52.5 sec/2.5e-4)
% 
t = data(1,imin:imax); % time (s)
icd = data(2,imin:imax); % desired current (A)
ic = data(3,imin:imax); % measured current (A)
xbd = data(4,imin:imax); % desired air gap (m)
xb = data(5,imin:imax); % measured air gap (m)
% 
% electromagnetic constant over run
Km_est = Mb*g*xb.^2./ic.^2; 
% smooth data
% Smooth measured data
smoothedDataIc = smoothdata(ic,"movmean");
smoothedDataXb = smoothdata(xb,"movmean");
smoothedDataKm = smoothdata(Km_est,"movmean");
% Create array for lookup table in Simulink
xbfit = linspace(5e-3,14e-3,13);
Km_xb_eqn = polyfit(smoothedDataXb,smoothedDataKm,3);
Km_xb_array = polyval(Km_xb_eqn,xbfit);
maglev_km = [xbfit; Km_xb_array]';
clear tmin tmax Ts imin imax t icd ic sbd xb Km_xb_eqn Km_xb_array ;
clear smoothedDataIc smoothedDataXb smoothedDataKm;
