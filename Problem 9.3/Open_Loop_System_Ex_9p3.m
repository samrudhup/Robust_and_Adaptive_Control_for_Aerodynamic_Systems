% Here there are 2 states to the x vector
%
% x = [phi, p]
%
%--------------------------------------------------------------------------
function dx = Open_Loop_System_Ex_9p3(t, x, A, B, theta1, theta2, ...
    theta3, theta4, theta5, theta6)

% Call command function
dela = 0;

% Compute basis functions (regressor vector) at x(t)
Phi = [abs(x(1))*x(2); abs(x(2))*x(2); x(1)^3];

% Compute f(x) as Theta^T*Phi
Theta = 1/theta6*[theta3; theta4; theta5];
f_of_x = Theta'*Phi;

% Specify the uncertainty Lambda
Lambda = theta6;

% Compute RHS of the system
dx = A*x + B*Lambda*( dela + f_of_x );
