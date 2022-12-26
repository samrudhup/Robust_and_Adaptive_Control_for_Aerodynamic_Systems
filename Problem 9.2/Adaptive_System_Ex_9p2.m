% Here there are 5 states to the x vector
%
% x = [q, q_ref, kq_est, kqcmd_est, theta_est]
%
%--------------------------------------------------------------------------
function dx = Adaptive_System_Ex_9p2(t, x, Mq, Md, a_ref, b_ref, ...
    gamma_q, gamma_qcmd, Gamma_theta)

dx = [0; 0; 0; 0; 0];

% Call command function
q_cmd = pitch_rate_command(t);

% Compute Phi(q)
Phiq = tanh(360/pi*x(1));

% Compute control signal
del = x(3)*x(1) + x(4)*q_cmd - x(5)*Phiq;

% Compute RHS of the system
dx(1) = Mq*x(1) + Md*(del + -0.01*Phiq);

% Compute RHS of the reference system
dx(2) = a_ref*x(2) + b_ref*q_cmd;

% Compute RHS of the adaptive laws
dx(3) = gamma_q*x(1)*(x(1) - x(2));
dx(4) = gamma_qcmd*q_cmd*(x(1) - x(2));
dx(5) = -Gamma_theta*Phiq*(x(1) - x(2));
