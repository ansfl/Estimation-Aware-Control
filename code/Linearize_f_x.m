function [x_t, u_t, J_A, J_B] = Linearize_f_x( )
% ------------------------ Description ------------------------ %
%                                                               %
%   Input  : NA                                                 %
%   Output : Linearized (the nonlinear) Dynamics                %
%                                                               %
% -------------------------- Content -------------------------- %

%   This function linearizes the nonlinear quadrotor dynamics   %

% ---------------------- State variables ---------------------- %
syms x_1 x_2 x_3 real                          % Position coordinates   (inertial)
syms x_4 x_5 x_6 real                      % Velocity coordinates   (body)
syms x_7 x_8 x_9 real                      % Euler angles           (inertial)
syms x_10 x_11 x_12 real                       % Angluar velocity       (body)

% --------------------- Control variables --------------------- %
syms u_1 u_2 u_3 u_4 real                  % Control variables (thrust, roll, pitch, yaw)
syms m g J_x J_y J_z real                  % System parameters
    
% -------------------- Aerodynamic Effects -------------------- %
syms c_Vx c_Vy c_Vz c_Wp c_Wq c_Wr real    % Aerodynamic damping coefficients
V_tot = real(sqrt(x_4^2 + x_5^2 + x_6^2)); % Nonlinear 3D velocity norm

% Full nonlinear body-frame drag forces
F_drag_B = [c_Vx*V_tot*x_4;  c_Vy*V_tot*x_5;  c_Vz*V_tot*x_6]; % Translational Drag
M_drag_B = [c_Wp*V_tot*x_10; c_Wq*V_tot*x_11; c_Wr*V_tot*x_12]; % Rotational    friction

X_1_3 = [x_1; x_2; x_3]; % NOTE: --,'-- denotes non-conjugate (REAL symbolic variable)
X_4_6 = [x_4; x_5; x_6];
X_7_9 = [x_7; x_8; x_9];
X_012 = [x_10; x_11; x_12];

% Body-to-Inertial Transformation R_B_I
[phi, tht, psi] = deal(x_7, x_8, x_9);                % Keep as is : to enable variables swap

R_B_I = [[cos(tht)*cos(psi), sin(phi)*sin(tht)*cos(psi) - cos(phi)*sin(psi), cos(phi)*sin(tht)*cos(psi) + sin(phi)*sin(psi)];
        [ cos(tht)*sin(psi), sin(phi)*sin(tht)*sin(psi) + cos(phi)*cos(psi), cos(phi)*sin(tht)*sin(psi) - sin(phi)*cos(psi)];
        [         -sin(tht),                              sin(phi)*cos(tht),                              cos(phi)*cos(tht)]];
R_I_B = R_B_I.';

% f_skew = @(v) [[0,-v(3),v(2)];[v(3),0,-v(1)];[-v(2),v(1),0]];

[e_x_I, e_y_I, e_z_I] = deal([1,0,0]', [0,1,0]', [0,0,1]');
[e_x_B, e_y_B, e_z_B] = deal(R_I_B*e_x_I, R_I_B*e_y_I, R_I_B*e_z_I);

% Body-rates to inertial-rates Transformation W_eta
W_eta = [[1, sin(phi)*tan(tht), cos(phi)*tan(tht)];
        [0,           cos(phi),         -sin(phi)];
        [0,  sin(phi)/cos(tht), cos(phi)/cos(tht)]];

J = diag([J_x, J_y, J_z]);

% --------- Dynamical system function ---------- %
dX_1_3 = R_B_I*X_4_6;
dX_4_6 = - g*e_z_B - cross(X_012,X_4_6) + (1/m)*([0;0;u_1] - F_drag_B); % Aerodynamics appended
dX_7_9 = W_eta*X_012; 
dX_012 = J^(-1)*( - cross(X_012, J*X_012) + [u_2, u_3, u_4]' - M_drag_B ); % With Coriolis terms

f_x = [dX_1_3; dX_4_6; dX_7_9; dX_012]; 
x_t = [X_1_3; X_4_6; X_7_9; X_012];
u_t = [u_1; u_2; u_3; u_4]; % NOTE: All inputs are already IN body frame !

% --- 1. Compute Jacobians Symbolically ---
[J_A, J_B] = deal(jacobian(f_x, x_t), jacobian(f_x, u_t));         

% Retrieve Numeric System Parameters
[m_s, g_s, J_s, c_Vs, c_Ws, ~] = sys_params();                     

% Re-declare raw scalar symbols to guarantee mapping locks
syms m g J_x J_y J_z c_Vx c_Vy c_Vz c_Wp c_Wq c_Wr real
keys    = [m, g, J_x, J_y, J_z, c_Vx,  c_Vy,  c_Vz,  c_Wp,  c_Wq,  c_Wr];
targets = [m_s, g_s, J_s(1,1), J_s(2,2), J_s(3,3), c_Vs(1), c_Vs(2), c_Vs(3), c_Ws(1), c_Ws(2), c_Ws(3)];

% Assigning numerical substitution
J_A = subs(J_A, keys, targets); 
J_B = subs(J_B, keys, targets);