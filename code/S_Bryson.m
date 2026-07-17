function [Q_ii, R_jj] = S_Bryson( )
% ------------------------ Description ------------------------ %
%                                                               %
%   Input  : NA                                                 %
%   Output : Q & R weights according to Bryson's rule           %
%                                                               %
% -------------------------- Content -------------------------- %

% --- Outer Loop: Translation (Smooth and Balanced) ---
x_pos = [1, 1, 1]*0.75;             % [m] Position tolerance (Weight = 0.44)
x_vel = [1, 1, 1]*0.75;             % [m/s] Planar velocity tracking (Weight = 6.25)

% --- Inner Loop: Rotation (Fast and Heavily Damped) ---
x_Eul = [1, 1, 1]*0.05;            % [rad] Tighten angles to ~8.5 deg (Weight = 44.4)
x_omg = [1, 1, 1]*0.05;             % [rad/s] Massive rate damping! (Weight = 1111)

% ---------------------------------- %
Q_ii = diag([x_pos, x_vel, x_Eul, x_omg])^(-2);

% --- Optimized Virtual Control Budget (Physics-Matched) ---
u_acc = [1, 1, 1]*25.0;            % [m/s^2] Linear acceleration budget is fine

% Differentiate angular acceleration capabilities by axis
u_phi = 10.0;                      % [rad/s^2] Roll acceleration budget
u_tht = 10.0;                      % [rad/s^2] Pitch acceleration budget
u_psi = 150.0;                      % [rad/s^2] Yaw acceleration budget (Heavily restricted!)

% ---------------------------------- %
R_jj = diag([u_acc, u_phi, u_tht, u_psi])^(-2);

% Good Heuristic range
% Q_ii = eye(12)*10;
% R_jj = eye(6)/5000;

% ---------- OLD: Pure LQG ---------- %
% % Acceptable (max) state deviations
% x_pos = [1 1 1/3]*0.5;                         % [m]
% x_vel = [1 1 1/2]*0.2;                         % [m/s]
% x_Eul = 0.15;                         % [rad]
% x_omg = 0.15;                         % [rad/s]
% % ---------------------------------- %
% Q_ii = diag([x_pos, x_vel, x_Eul*I_3, x_omg*I_3])^(-2);
% % Q_ii = diag([x_pos*I_3, x_vel*I_3, x_Eul*I_3, x_omg*I_3])^(-2);
% 
% % Acceptable (max) control deviations
% u_z = 15.0;                          % [N] T_max = mg/2 [N]
% u_p = 0.015;                           % [N*m] 0.015
% u_q = 0.015;                           % [N*m] 0.015
% u_r = 0.05;                           % [N*m] 0.05
% % ---------------------------------- %
% R_jj = diag([u_z, u_p, u_q, u_r])^(-2);