% ------------------------ Description ------------------------ %
%                                                               %
%   Input  : N/A                                                %
%   Output : State vector solution                              %
%                                                               %
% -------------------------- Content -------------------------- %

for k=2:(length(tt))
    % -------------- Error module ------------- % (Estimated tracking error)
    % Compute raw error and apply the heading-error wrap to prevent polar spikes (360-degree yaw discontinuities)
    e_r(:,k-1) = x_e(:,k-1) - x_ref(:,k-1); 
    e_r(9,k-1) = atan2(sin(x_e(9,k-1) - x_ref(9,k-1)), cos(x_e(9,k-1) - x_ref(9,k-1)));

    % -------------- Control module ------------- % (x_k == GT | x_e == x^est)
    [F_k, G_k] = J_x_u(x_e(:,k-1), u_out_k(:,k-1)); % Compute state & input Jacobians
    u_INDI_k = u_INDI(e_r(:,k-1), P_k, x_e(:,k-1), u_out_k(:,k-1), K, G_k);   % Pure INDI law (accelerations)

    % Enforce physical saturation and actuator dynamics time-constant delay
    u_cmd_k(:,k) = u_sat(u_INDI_k);                                           % Pure command inputs (u[1:4])
    u_out_k(:,k) = u_out_k(:,k-1) + t_tau * (u_cmd_k(:,k) - u_out_k(:,k-1) ); % Physical actuation (subject to \tau)
    
    % --------- Nonlinear (true) process -------- %
    x_k(:,k) = f_RK(@f_x_u, x_k(:,k-1), u_out_k(:,k), dt) + w_k();% (EKF -- GT)
    
    % ------- Nonlinear state estimation -------- %
    x_e(:,k) = f_RK(@f_x_u, x_e(:,k-1), u_out_k(:,k), dt); % (EKF -- Estimated)
    P_k = F_k*P_k*F_k' + W;                       % Error state covariance (open-loop)
    
    % ------------- Position Update ------------- %
    if mod(k,u_2_p) == 0
        C_k = h_x(x_idx, x_dim);                  % Projection: position, vel, gyros
        y_meas = C_k*x_k(:,k) + v_k();          % Outputs (Noisy measurements)
        y_pred = C_k*x_e(:,k);                  % Predicted measurements
        y_res  = y_meas - y_pred;                 % Measurements residual

        [x_e(:,k), P_k] = KF_update(x_e(:,k), P_k, y_res, C_k, V); % Kalman filter (loop-closure)
        y_k(x_idx,k) = y_meas;
    end
    x_std(:,k) = sqrt(diag(P_k));                 % Save standard deviation

    % Wrap final output orientations strictly for safe data storage/plots
    [x_k(9,k), x_e(9,k)] = deal(wrapToPi(x_k(9,k)), wrapToPi(x_e(9,k)));

    % Loop Termination & Divergence Guardrails
    if norm(e_r(1:3,k), 1) > 10.0                  
        error('Simulation Terminated: Positional error exceeded 15m limit at step %d.', k);
    elseif ~isreal(x_e) || ~isreal(x_k) || ~isreal(e_r) 
        error('Simulation Terminated: Complex numbers detected at step %d, t = %.3f s. Filter diverged.', k, tt(k));
    end
end