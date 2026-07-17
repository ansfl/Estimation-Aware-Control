function [W, V] = S_Cov( dt )
% ------------------------ Description ------------------------ %
%                                                               %
%   Input  : NA                                                 %
%   Output : W & V covariance matrices                          %
%                                                               %
% -------------------------- Content -------------------------- %

I_1 = ones(1,3); freq = 1/dt;

% Process noise parameters
sig_f = 0.002*sqrt(freq);                % Accelerometer noise density [m/s^2/Hz^.5]
sig_w = 0.001*sqrt(freq);                % Angular rate process noise  [rad/s/Hz^.5]
% ------------------------------------- %
% W = diag([(1/3)*sig_f^2*dt*I_1, sig_f^2*dt*I_1, sig_w^2*dt*I_1, sig_w^2*dt*I_1]);
W = diag([1e-6*I_1, 1e-6*I_1, 1e-6*I_1, 5e-5*I_1])/100;          % Good range

% Measurement noise parameters
sig_GPS  = (1/1000)*sqrt(freq);         % GPS SD [m/Hz^.5] (Randowm walk of 1 m per each second)
sig_vel = 0.05;                         % Velocity precision [m/s]
sig_Eul = 0.05;                         % AHRS + Magnetometer precision [rad]
sig_omg = 0.05;                         % Gyroscope precision [rad/s]
% ------------------------------------- %
% V = diag([sig_GPS^2*I_1, sig_vel^2*I_1, sig_Eul^2*I_1, sig_omg^2*I_1]);
V = diag([1e-6*I_1, 5e-6*I_1, 5e-6*I_1, 1e-7*I_1])/100;          % Good range