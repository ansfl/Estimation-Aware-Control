function U_out = u_sat( K_err )
% ------------------------ Description ------------------------ %
%                                                               %
%   Input  : Desired control input                              %
%   Output : Constrained & feasible actuation                   %
%                                                               %
% -------------------------- Content -------------------------- %

% Apply mixer matrix + entire propeller dynamics ... u_cmd --> u_out
[u_1_in, u_2_in, u_3_in, u_4_in] = deal(K_err(1), K_err(2), K_err(3), K_err(4));
L = 0.15; [~,~,~,~,~,T_max] = sys_params();

% ------------ Actuators boundaries ----------- %
max_thrust = T_max;                             % Maximum thrust force
[u_1_min, u_1_max] = deal(0.5, max_thrust);     % Min and max thrust forces (3847 < w < 8,600) [RPM]
u_2_max = (u_1_max - u_1_min)*(L/2)*sin(pi/4);  % Calculate Roll and Pitch Limits (X-Configuration Geometric Leverage)
u_3_max = u_2_max;                              
u_4_max = u_2_max*0.15;                         % A safe, realistic aerodynamic rule-of-thumb ratio

u_1_out = max(+u_1_min, min(u_1_in, u_1_max));  % Thrust range (positive only)
u_2_out = max(-u_2_max, min(u_2_in, u_2_max));  % Rolling  range (+/-)
u_3_out = max(-u_3_max, min(u_3_in, u_3_max));  % Pitching range (+/-)
u_4_out = max(-u_4_max, min(u_4_in, u_4_max));  % Yawing   range (+/-)

U_out = [u_1_out, u_2_out, u_3_out, u_4_out];
