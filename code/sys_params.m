function [m, g, J, c_V, c_W, T_max] = sys_params()
% ------------------------ Description ------------------------ %
%                                                               %
%   Input  : N/A                                                %
%   Output : Physical paramters                                 %
%                                                               %
% -------------------------- Content -------------------------- %

m = 0.9689; L = 0.15; R = 0.125; g = 9.81;                      % Physical parameters
rho = 1.225; % eta_eff = 0.8; A_R = pi*R^2; 
[J_xx, J_yy, J_zz] = deal(0.01587, 0.01405, 0.02790);           % Moment of Inertia
J = diag([J_xx, J_yy, J_zz]);

RPM = 8000; RPS = RPM/60;                                       % Max operating frequency
C_T = (m*g/4) / ( rho*pi*(RPS*2*pi)^2*R^4 );                    % Thrust coefficient (unitless)
C_M = (0.025) / ( rho*pi*(RPS*2*pi)^2*R^5 );                    % Moment coefficient (unitless)
k_T = rho*pi*C_T*R^4;                                           % Thrust constant <==> k_T = 0.5*rho*A_R*C_T*R^2;
k_M = rho*pi*C_M*R^5;                                           % Moment constant <==> k_M = 0.5*rho*A_R*C_M*R^3;
T_max = 4*2*k_T * (RPS*2*pi)^2;                                 % Thrust limit

[A_xy, A_z] = deal(0.010, 0.055);                               % Cross section area [m^2]
c_V = 0.5*rho*[A_xy; A_xy; 1.5*A_z];                            % Translational coeff
c_W = 0.5*rho*[A_xy; A_xy; 0.3*A_z]*L^2;                        % Rotational    coeff