function [F_v, G_v] = J_INDI( )
% ------------------------ Description ------------------------ %
%                                                               %
%   Input  : N/A                                                %
%   Output : State and Input Jacobians for INDI control         %
%                                                               %
% -------------------------- Content -------------------------- %

% Create the Ideal 6-Axis Virtual Kinematic Model (Sized for 6 acceleration inputs)
F_v = zeros(12,12);
F_v(1:3, 4:6)   = eye(3);   % d/dt(Inertial position) Body Velocity
F_v(7:9, 10:12) = eye(3);   % d/dt(Euler Angles) Body Angular Rates

% Kinetics (Virtual Acceleration Drives)
G_v = zeros(12,6);
G_v(4:6, 1:3)   = eye(3);   % Virtual input 1:3 drives body linear acceleration (du, dv, dw)
G_v(10:12, 4:6) = eye(3);   % Virtual input 4:6 drives body angular acceleration (dp, dq, dr)
end