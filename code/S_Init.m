% ------------------------ Description ------------------------ %
%                                                               %
%   Input  : N/A                                                %
%   Output : Initial conditions                                 %
%                                                               %
% -------------------------- Content -------------------------- %

% Get parameters
O_3 = [0,0,0]; I_1 = [1,1,1];                % Initial Conditions 
[m, g, J, c_V, c_W, T_max] = sys_params();   % Get params
[W, V] = S_Cov( dt );                        % Covariance matrices (process & measurement)
[x_t, u_t, J_A, J_B] = Linearize_f_x( );     % Linearized system dynamics

% Pre-allocation of Jacobians (removing symbolic dependence)
matlabFunction(J_A, 'File', 'calc_JA_fast', 'Vars', {x_t, u_t});
matlabFunction(J_B, 'File', 'calc_JB_fast', 'Vars', {x_t, u_t});

syms p_x p_y p_z;
x_eq = [p_x; p_y; p_z; O_3'; O_3'; O_3'];    % Equilibrium point: States
u_eq = [0, 0, 0, 0, 0, 0]';                  % Equilibrium point: INDI inputs

% Assign LQR weights to INDI control effectiveness (penalizing the 6 virtual accelerations)
[F_v, G_v] = J_INDI( );
K = lqr(F_v, G_v, Q_lqr, R_lqr);                % Compute optimal LQR gain matrix

% Assign state and input vector
x_dim = size(J_A, 1);                           % Number of states
u_dim = size(J_B, 2);                           % Number of inputs

% Initialization of control vectors
u_out_k = zeros(u_dim, length(tt)); 
u_cmd_k = zeros(u_dim, length(tt));
f_IMU   = zeros(1, length(tt));                 % Acceleration array

% Initialization of state vectors
x_0   = x_ref(:,1);                          % Initial point (x_Ref_0)
x_std = zeros(x_dim, length(tt)); y_k = x_std;
x_k = x_std; x_k(:,1) = x_0;
x_e = x_std; x_e(:,1) = x_0;
e_r = x_std; e_r(:,1) = x_0;

x_idx = (1:x_dim)';                             % Full obsverability
v_idx = [4,5,6]';
w_k = @() randn( [x_dim, 1] ).*sqrt(diag(W));
v_k = @() randn( [x_dim, 1] ).*sqrt(diag(V));

% Solution loop
F_k = expm(F_v*dt); G_k = G_v*dt;               % Discrete-time solution
P_k = 0.1*eye(x_dim);                           % Initialize error covariance