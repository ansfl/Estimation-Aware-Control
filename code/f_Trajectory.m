function X_Ref = f_Trajectory(tt, dt, w_f)
% ------------------------ Description ------------------------ %
%                                                               %
%   Input  : Time vector, Time increment, Velocity              %
%   Output : Reference trajectory generation                    %
%                                                               %
% -------------------------- Content -------------------------- %

g = 9.81;                                       % Gravity constant

% Figure-8 Trajectory Tuning Parameters
[w_x, w_y, w_z] = deal(w_f, 2*w_f, 1*w_f); % Angular frequencies
[A_x, A_y, A_z] = deal(5.0, 2.0, 0.75); % Trajectory dimensions [m]
% [A_x, A_y, A_z] = deal(5.0, 2.5, 1.0); % Trajectory dimensions [m]

% Inertial Kinematics 
P_I = [A_x*sin(w_x*tt); A_y*sin(w_y*tt); A_z*sin(w_z*tt)];
V_I = [A_x*w_x*cos(w_x*tt); A_y*w_y*cos(w_y*tt); A_z*w_z*cos(w_z*tt)];
A_I = [-A_x*(w_x^2)*sin(w_x*tt); -A_y*(w_y^2)*sin(w_y*tt); -A_z*(w_z^2)*sin(w_z*tt)];

% Desired yaw: tangent to horizontal velocity
Ref_psi = unwrap(atan2(V_I(2,:), V_I(1,:)));

% Rotate inertial acceleration into yaw-aligned frame
ax_B =  cos(Ref_psi).*A_I(1,:) + sin(Ref_psi).*A_I(2,:);
ay_B = -sin(Ref_psi).*A_I(1,:) + cos(Ref_psi).*A_I(2,:);
az_B = A_I(3,:);

% Reference roll and pitch (Keep continuous for numerical gradients)
Ref_phi = atan2(-ay_B, az_B + g);
Ref_tht = atan2( ax_B, sqrt(ay_B.^2 + (az_B + g).^2) );

% Numerical Differentiation of Continuous Angles
d_phi = gradient(Ref_phi, dt);
d_tht = gradient(Ref_tht, dt);
d_psi = gradient(Ref_psi, dt);
W_I   = [d_phi; d_tht; d_psi];

% Frame Transformations Loop (Velocities & Angular Rates)
N = length(tt); V_B = zeros(3, N); W_B = zeros(3, N); 

for k = 1:N
    phi = Ref_phi(k); tht = Ref_tht(k); psi = Ref_psi(k); % Assign Euler angles (wrapped)
    
    % Reconstruct the Body-to-Inertial Rotation Matrix (R_B_I) using wrapped angles
    R_B_I = [[cos(tht)*cos(psi), sin(phi)*sin(tht)*cos(psi) - cos(phi)*sin(psi), cos(phi)*sin(tht)*cos(psi) + sin(phi)*sin(psi)];
            [ cos(tht)*sin(psi), sin(phi)*sin(tht)*sin(psi) + cos(phi)*cos(psi), cos(phi)*sin(tht)*sin(psi) - sin(phi)*cos(psi)];
            [         -sin(tht),                              sin(phi)*cos(tht),                              cos(phi)*cos(tht)]];
    R_I_B = R_B_I.'; 
    
    % Transform Velocity: V^B = R_I_B*V^I
    V_B(:, k) = R_I_B*V_I(:, k);
    
    % Convert Euler rates to body angular rates (p, q, r)
    T_I_B = [1,  0,        -sin(tht);
             0,  cos(phi),  sin(phi)*cos(tht);
             0, -sin(phi),  cos(phi)*cos(tht)];
    W_B(:,k)  = T_I_B*W_I(:,k);
end

% Wrap the output orientations identically
Eul = [wrapToPi(Ref_phi); wrapToPi(Ref_tht); wrapToPi(Ref_psi)]; 

% Pack the structural components into the unified 12-state vector
X_Ref = [P_I; V_B; Eul; W_B];
end