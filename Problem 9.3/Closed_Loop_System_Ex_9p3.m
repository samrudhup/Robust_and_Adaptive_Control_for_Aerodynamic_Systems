% Here there are 2 states to the x vector
%
% x = [phi, p]
%
%--------------------------------------------------------------------------
function dx = Closed_Loop_System_Ex_9p3(t, x, A, B, theta1, theta2, ...
    theta3, theta4, theta5, theta6, P, Gamma_x, Gamma_r, Gamma_theta, ...
    A_ref, B_ref)

% Preallocate dx
dx = [0; 0; 0; 0; 0; 0; 0; 0; 0; 0];

% Compute basis functions at x(t)
Phi = [abs(x(1))*x(2); abs(x(2))*x(2); x(1)^3];

% Compute f(x) as Theta^T*Phi
Theta = 1/theta6*[theta3; theta4; theta5];
f_of_x = Theta'*Phi;

% Specify the uncertainty Lambda
Lambda = theta6;

% Compute current bank angle command
phi_cmd_rad = bank_angle_command(t)*pi/180; 

% Compute the RHS of the reference system
dx(9:10) = A_ref*x(9:10) + B_ref*phi_cmd_rad;

% Compute the error, e = x - x_ref
err = x(1:2) - x(9:10);

% Compute RHS of MIMO MRAC laws
dx(3:4) = -Gamma_x*x(1:2)*err'*P*B;
dx(5)   = -Gamma_r*phi_cmd_rad*err'*P*B;
dx(6:8) =  Gamma_theta*Phi*err'*P*B;

% Compute control signal
dela = x(3:4)'*x(1:2) + x(5)*phi_cmd_rad - x(6:8)'*Phi;

% Compute RHS of the system
dx(1:2) = A*x(1:2) + B*Lambda*( dela + f_of_x );
