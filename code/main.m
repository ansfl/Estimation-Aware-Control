%% =========================================================================
%  QUADROTOR FLIGHT DYNAMICS: SIMULATION, ESTIMATION & TRACKING ANALYSIS
%  =========================================================================
%  This script simulates a 12-state quadrotor system, runs state estimation,
%  and plots the tracking performance against a reference trajectory.
%  =========================================================================

clc; clear; close all;

%% -------------------------------------------------------------------------
%  1. ENVIRONMENT & PARAMETER CONFIGURATION
%  -------------------------------------------------------------------------
S_Clear_Canvas; % Clear canvas and prepare for presentation graphics

% --- Temporal Domain ---
dt    = 0.001;                 % Simulation step size [s]
t_f   = 25.0;                  % Final simulation time [s]
tt    = 0:dt:t_f;              % Time vector
t_len = length(tt);            % Number of steps

% --- System Settings ---
u_2_p      = 1;                % Update-to-prediction ratio (Must be an INTEGER)
time_const = 5*dt;             % Control time constant
t_tau      = dt / time_const;  % Discretized control rate

% --- Unit Conversions ---
Rad_2_deg = 180/pi;            % Radians to Degrees conversion factor
Rad_2_RPM = 30/pi;             % Radians/sec to RPM conversion factor

%% -------------------------------------------------------------------------
%  2. REFERENCE TRAJECTORY GENERATION
%  -------------------------------------------------------------------------
w_f   = 0.4;                            % Flight (total) angular rate [rad/s]
x_ref = f_Trajectory(tt, dt, w_f);      % Reference states [12 x t_len]

% Load controller design weights (Bryson's Rule)
[Q_lqr, R_lqr] = S_Bryson();             

% Display nominal mission velocity profile
fprintf('|| v^B || = %.2f [m/s]\n', norm(x_ref(4:6,:), 1));

%% -------------------------------------------------------------------------
%  3. SOLVE DYNAMICS & STATE ESTIMATION
%  -------------------------------------------------------------------------
S_Init;         % Initialize state vectors, covariances, and graphics handles
S_Solve;        % Run the recursive numerical integration / filtering loop

% Assign local analytical aliases
X_est = x_e;    % Filter-estimated state vector history
X__GT = x_k;    % Ground-Truth (true system states) history

% Compensate for indexing offset in standard deviation array to prevent zero division
x_std(:,1) = x_std(:,2); 

%% -------------------------------------------------------------------------
%  4. VISUALIZATION: 3D TRAJECTORY COMPARISON
%  -------------------------------------------------------------------------
l_w   = 2.5;                    % Main line width
f_col = [0.8 0.4 0.1];          % Accent color (Orange/Brown)
f_s   = 16;                     % Label font size
s_p   = 250;                    % Plot downsampling factor (speed up rendering)
s_end = round(t_len * 0.7);     % Plot up to 70% of total flight time

Fig([1900 700 550 450]);        % Initialize window canvas
hold on; grid on;

% Plot Trajectories
plot3(X__GT(1, 1:s_p:s_end), X__GT(2, 1:s_p:s_end), X__GT(3, 1:s_p:s_end), ...
      'LineWidth', l_w); 
plot3(X_est(1, 1:s_p:s_end), X_est(2, 1:s_p:s_end), X_est(3, 1:s_p:s_end), ...
      'Color', f_col, 'LineWidth', l_w, 'LineStyle', '--'); 
plot3(x_ref(1, 1:s_p:s_end), x_ref(2, 1:s_p:s_end), x_ref(3, 1:s_p:s_end), ...
      'Color', [1, 1, 1]*0.3, 'LineWidth', l_w, 'LineStyle', '--'); 

% Plot starting point marker
scatter3(x_ref(1,1), x_ref(2,1), x_ref(3,1), ...
         'SizeData', 80, 'MarkerEdgeColor', 'k', 'MarkerFaceColor', [0.05, 0.85, 0.25]);

% Labels and Camera Properties
xlabel('$\xi_x^I$', 'Interpreter', 'latex', 'FontSize', f_s); 
ylabel('$\xi_y^I$', 'Interpreter', 'latex', 'FontSize', f_s); 
zlabel('$\xi_z^I$', 'Interpreter', 'latex', 'FontSize', f_s);
view([-55, 15]); 
set(gca, 'DataAspectRatio', [1 1.5 1.0]);

legend({'$\boldmath{x}_{GT} \hspace{12mm} $', '$\hat{\boldmath{x}}$'}, ...
       'Interpreter', 'latex', 'Orientation', 'horizontal', ...
       'Position', [0.1, 0.7, 0.37, 0.04], 'FontSize', 12);

%% -------------------------------------------------------------------------
%  5. VISUALIZATION: COMPREHENSIVE 12-STATE TIME-SERIES
%  -------------------------------------------------------------------------
Fig([2450 700 550 450]); 
f_s = 12;

% --- Subplot Block 1: Position States (\xi^I) ---
subplot(4, 3, 1); hold on; 
plot(tt, X__GT(1,:)); plot(tt, X_est(1,:), 'Color', f_col, 'LineWidth', l_w, 'LineStyle', '--');
plot(tt, x_ref(1,:), 'Color', 'k', 'LineWidth', l_w/2, 'LineStyle', '-.');
ylabel('$\xi_x^I(t)$ [m]', 'Interpreter', 'latex', 'FontSize', f_s);

subplot(4, 3, 2); hold on;
plot(tt, X__GT(2,:)); plot(tt, X_est(2,:), 'Color', f_col, 'LineWidth', l_w, 'LineStyle', '--');
plot(tt, x_ref(2,:), 'Color', 'k', 'LineWidth', l_w/2, 'LineStyle', '-.');
ylabel('$\xi_y^I(t)$ [m]', 'Interpreter', 'latex', 'FontSize', f_s);

subplot(4, 3, 3); hold on;
plot(tt, X__GT(3,:)); plot(tt, X_est(3,:), 'Color', f_col, 'LineWidth', l_w, 'LineStyle', '--');
plot(tt, x_ref(3,:), 'Color', 'k', 'LineWidth', l_w/2, 'LineStyle', '-.');
ylabel('$\xi_z^I(t)$ [m]', 'Interpreter', 'latex', 'FontSize', f_s);

% --- Subplot Block 2: Translational Velocity (v^B) ---
subplot(4, 3, 4); hold on; 
plot(tt, X__GT(4,:)); plot(tt, X_est(4,:), 'Color', f_col, 'LineWidth', l_w, 'LineStyle', '--');
plot(tt, x_ref(4,:), 'Color', 'k', 'LineWidth', l_w/2, 'LineStyle', '-.');
ylabel('$v_x^B(t)$ [m/s]', 'Interpreter', 'latex', 'FontSize', f_s);

subplot(4, 3, 5); hold on;
plot(tt, X__GT(5,:)); plot(tt, X_est(5,:), 'Color', f_col, 'LineWidth', l_w, 'LineStyle', '--');
plot(tt, x_ref(5,:), 'Color', 'k', 'LineWidth', l_w/2, 'LineStyle', '-.');
ylabel('$v_y^B(t)$ [m/s]', 'Interpreter', 'latex', 'FontSize', f_s);

subplot(4, 3, 6); hold on;
plot(tt, X__GT(6,:)); plot(tt, X_est(6,:), 'Color', f_col, 'LineWidth', l_w, 'LineStyle', '--');
plot(tt, x_ref(6,:), 'Color', 'k', 'LineWidth', l_w/2, 'LineStyle', '-.');
ylabel('$v_z^B(t)$ [m/s]', 'Interpreter', 'latex', 'FontSize', f_s);

% --- Subplot Block 3: Attitude Euler Angles (\phi, \theta, \psi) ---
subplot(4, 3, 7); hold on; 
plot(tt, X__GT(7,:)*Rad_2_deg); plot(tt, X_est(7,:)*Rad_2_deg, 'Color', f_col, 'LineWidth', l_w, 'LineStyle', '-.');
plot(tt, x_ref(7,:)*Rad_2_deg, 'Color', 'k', 'LineWidth', l_w/2, 'LineStyle', '-.');
ylabel('$\phi(t)$ [$^\circ$]', 'Interpreter', 'latex', 'FontSize', f_s);

subplot(4, 3, 8); hold on;
plot(tt, X__GT(8,:)*Rad_2_deg); plot(tt, X_est(8,:)*Rad_2_deg, 'Color', f_col, 'LineWidth', l_w, 'LineStyle', '-.');
plot(tt, x_ref(8,:)*Rad_2_deg, 'Color', 'k', 'LineWidth', l_w/2, 'LineStyle', '-.');
ylabel('$\theta(t)$ [$^\circ$]', 'Interpreter', 'latex', 'FontSize', f_s);

subplot(4, 3, 9); hold on;
plot(tt, X__GT(9,:)*Rad_2_deg); plot(tt, X_est(9,:)*Rad_2_deg, 'Color', f_col, 'LineWidth', l_w, 'LineStyle', '-.');
plot(tt, x_ref(9,:)*Rad_2_deg, 'Color', 'k', 'LineWidth', l_w/2, 'LineStyle', '-.');
ylabel('$\psi(t)$ [$^\circ$]', 'Interpreter', 'latex', 'FontSize', f_s);

% --- Subplot Block 4: Body Angular Rates (\omega^B) ---
subplot(4, 3, 10); hold on; 
plot(tt, X__GT(10,:)*Rad_2_deg); plot(tt, X_est(10,:)*Rad_2_deg, 'Color', f_col, 'LineWidth', l_w, 'LineStyle', '--');
plot(tt, x_ref(10,:)*Rad_2_deg, 'Color', 'k', 'LineWidth', l_w/2, 'LineStyle', '-.');
ylabel('$\omega_x^B(t)$ [$^\circ$/s]', 'Interpreter', 'latex', 'FontSize', f_s);
xlabel('Time [s]', 'Interpreter', 'latex', 'FontSize', f_s);

subplot(4, 3, 11); hold on;
plot(tt, X__GT(11,:)*Rad_2_deg); plot(tt, X_est(11,:)*Rad_2_deg, 'Color', f_col, 'LineWidth', l_w, 'LineStyle', '--');
plot(tt, x_ref(11,:)*Rad_2_deg, 'Color', 'k', 'LineWidth', l_w/2, 'LineStyle', '-.');
ylabel('$\omega_y^B(t)$ [$^\circ$/s]', 'Interpreter', 'latex', 'FontSize', f_s);
xlabel('Time [s]', 'Interpreter', 'latex', 'FontSize', f_s);

subplot(4, 3, 12); hold on;
plot(tt, X__GT(12,:)*Rad_2_deg); plot(tt, X_est(12,:)*Rad_2_deg, 'Color', f_col, 'LineWidth', l_w, 'LineStyle', '--');
plot(tt, x_ref(12,:)*Rad_2_deg, 'Color', 'k', 'LineWidth', l_w/2, 'LineStyle', '-.');
ylabel('$\omega_z^B(t)$ [$^\circ$/s]', 'Interpreter', 'latex', 'FontSize', f_s);
xlabel('Time [s]', 'Interpreter', 'latex', 'FontSize', f_s);

% --- Shared Axis Scaling & Formatting (Vectorized) ---
y_all = X__GT(10:12, :); 
ylim_shared = [min(y_all(:))*Rad_2_deg, max(y_all(:))*Rad_2_deg];
arrayfun(@(i) set(subplot(4,3,i), 'YLim', ylim_shared), 10:12);
arrayfun(@(i) grid(subplot(4, 3, i), 'on'), 1:12); 

legend({'$\boldmath{x}_{GT} \hspace{12mm} $', '$\hat{\boldmath{x}}$'}, ...
       'Interpreter', 'latex', 'Orientation', 'horizontal', ...
       'Position', [0.33, 0.95, 0.37, 0.04], 'FontSize', 12);

%% -------------------------------------------------------------------------
%  6. VISUALIZATION: ESTIMATION AND TRACKING ERRORS
%  -------------------------------------------------------------------------
e_r(:, end) = x_e(:, end) - x_ref(:, end); % Handle index margin

% Calculate Norm Errors across functional state groups (Position, Vel, Attitude, Rates)
err_trk = [vecnorm(e_r(1:3,:)); vecnorm(e_r(4:6,:)); vecnorm(e_r(7:9,:)); vecnorm(e_r(10:12,:))];
err_est = [vecnorm(X__GT(1:3,:) - X_est(1:3,:)); vecnorm(X__GT(4:6,:) - X_est(4:6,:)); ...
           vecnorm(X__GT(7:9,:) - X_est(7:9,:)); vecnorm(X__GT(10:12,:) - X_est(10:12,:))];

l_w  = 1.75; 
f_s  = 12; 
c_gr = [0.8 0.4 0.1];

Fig([3100 700 550 450]);

% --- Position Errors ---
subplot(4, 2, 1); hold on; grid on;
plot(tt, err_est(1,:), 'LineWidth', l_w/2, 'Color', c_gr);
ylabel('$\| \tilde{x}_{\xi} (t) \|$ [m]', 'Interpreter', 'latex', 'FontSize', f_s);
ylim([0, 0.0015]);

subplot(4, 2, 2); hold on; grid on;
plot(tt, err_trk(1,:), 'LineWidth', l_w, 'Color', c_gr);
ylabel('$\| \hat{\epsilon}_{\xi} (t) \|$ [m]', 'Interpreter', 'latex', 'FontSize', f_s);

% --- Velocity Errors ---
subplot(4, 2, 3); hold on; grid on;
plot(tt, err_est(2,:), 'LineWidth', l_w/2, 'Color', c_gr); 
ylabel('$\| \tilde{x}_{v} (t) \|$ [m/s]', 'Interpreter', 'latex', 'FontSize', f_s);
ylim([0, 0.002]);

subplot(4, 2, 4); hold on; grid on;
plot(tt, err_trk(2,:), 'LineWidth', l_w, 'Color', c_gr); 
ylabel('$\| \hat{\epsilon}_{v} (t) \|$ [m/s]', 'Interpreter', 'latex', 'FontSize', f_s);

% --- Attitude Errors ---
subplot(4, 2, 5); hold on; grid on;
plot(tt, (err_est(3,:))*Rad_2_deg, 'LineWidth', l_w/2, 'Color', c_gr); 
ylabel('$\| \tilde{x}_{\Theta} (t) \|$ [$^\circ$]', 'Interpreter', 'latex', 'FontSize', f_s);
ylim([0, 0.15]);

subplot(4, 2, 6); hold on; grid on;
plot(tt, (err_trk(3,:))*Rad_2_deg, 'LineWidth', l_w, 'Color', c_gr); 
ylabel('$\| \hat{\epsilon}_{\Theta} (t) \|$ [$^\circ$]', 'Interpreter', 'latex', 'FontSize', f_s);

% --- Angular Rate Errors ---
subplot(4, 2, 7); hold on; grid on;
plot(tt, err_est(4,:)*Rad_2_deg, 'LineWidth', l_w/2, 'Color', c_gr); 
ylabel('$\| \tilde{x}_{\omega} (t) \|$ [$^\circ$/s]', 'Interpreter', 'latex', 'FontSize', f_s);
xlabel('Time [s]', 'Interpreter', 'latex', 'FontSize', f_s); 
ylim([0, 0.05]);

subplot(4, 2, 8); hold on; grid on;
plot(tt, err_trk(4,:)*Rad_2_deg, 'LineWidth', l_w, 'Color', c_gr); 
ylabel('$\| {\epsilon}_{\omega} (t) \|$ [$^\circ$/s]', 'Interpreter', 'latex', 'FontSize', f_s);
xlabel('Time [s]', 'Interpreter', 'latex', 'FontSize', f_s); 
ylim([0, 2]);