function [J_A_out, J_B_out] = J_x_u(x_0, u_0)
% ------------------------ Description ------------------------ %
%
%   Input  : J_A, J_B (Symbolic), x_t, u_t (Sym vars), x_0, u_0 (Numeric coordinates)
%   Output : Linearized Dynamics (Numerical Matrices)
%
% ------------------------------------------------------------- %


% Compute Matrix A (Always executed)
% Evaluates the generated file instantly using raw floating-point math
J_A_out = calc_JA_fast(x_0, u_0);
J_A_out = double(J_A_out);

    % Compute Matrix B (Only executed if explicitly requested)
    if nargout > 1
        J_B_out = calc_JB_fast(x_0, u_0);
        J_B_out = double(J_B_out);
    else
        J_B_out = []; 
    end
end
