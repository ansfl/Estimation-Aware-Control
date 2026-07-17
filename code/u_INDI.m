function u_out = u_INDI(err_k, P_k, x_e_prev, u_out_prev, K_nu, G_full)
% ------------------------ Description ------------------------ %
%                                                               %
%   Input  : Error, state estimate, and control elements        %
%   Output : Updated INDI Control increment                     %
%                                                               %
% -------------------------- Content -------------------------- %
% Define indices for linear and angular accelerations
acc_indices = [4, 5, 6, 10, 11, 12];

% Evaluate current system acceleration state derivatives
dot_x_e = f_x_u(x_e_prev, u_out_prev);              % Full state derivative
dot_x_e = dot_x_e(acc_indices);                     % Accelerations only
P_k_a   = P_k(acc_indices, acc_indices);                         % Accelerations only

% Compute virtual acceleration target vector via LQR
nu = -K_nu * err_k;

% Slice full input Jacobian to match acceleration rows
G_v = G_full(acc_indices, :);

% Calculate the normalized correlation matrix (The EA mechanism)
D_k_inv_sqrt = diag(1 ./ sqrt(diag(P_k_a)));
P_k_N = D_k_inv_sqrt * P_k_a * D_k_inv_sqrt;
d_eta = ( eye(size(G_v,1)) - P_k_N ) * dot_x_e;

% Calculate physical control increment via pseudo-inverse
% d_a = (nu - dot_x_e);                       % Regular INDI
d_a = (nu - dot_x_e + d_eta);                 % EA-INDI (Estimation-Aware mechanism)
d_u = pinv(G_v) * d_a;

% Integrate increment onto previous control state
u_out = u_out_prev + d_u;
end