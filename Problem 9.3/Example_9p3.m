%% Example 9.3 Lavretsky and Wise

% Plant, states are phi and p
theta1 = -0.018;
theta2 =  0.015;
theta3 = -0.062;
theta4 =  0.009;
theta5 =  0.021;
theta6 =  0.750;

A = [0 1; theta1 theta2];
B = [0; 1];
Lambda = theta6;
Theta  = [theta3 theta4 theta5]/theta6;

%% Part 1a: Explore the dynamics near the origin by first integrating the
% open loop ode system for various initial conditions
%--------------------------------------------------------------------------
options = odeset('reltol', 1e-10, 'abstol', 1e-10);
[t1, x1] = ode45(@Open_Loop_System_Ex_9p3, [0 800], [5*pi/180 0], ...
    options, A, B, theta1, theta2, theta3, theta4, theta5, theta6);

[t2, x2] = ode45(@Open_Loop_System_Ex_9p3, [0 500], [50*pi/180 0], ...
    options, A, B, theta1, theta2, theta3, theta4, theta5, theta6);

% Part 1b: Explore the dynamics near the origin by second plotting the ode
% system trajectories for each initial condition
%--------------------------------------------------------------------------
figure(1)
plot(x1(:,1)*180/pi, x1(:,2)*180/pi, 'b', 'linewidth', 2); hold on 
plot(x2(:,1)*180/pi, x2(:,2)*180/pi, 'r', 'linewidth', 2); 
xlabel('Bank Angle [deg]','fontsize',14);
ylabel('Roll Rate [deg/s]','fontsize',14);
set(gca,'fontsize',14);
set(gcf,'color','w');
grid on 

%% Part 2a: Implement the closed loop adaptive controller
%--------------------------------------------------------------------------

% Initial condition
phi_rad   = 10*pi/180;
p_rps     = 0;
Kxhat     = [0; 0];
Krhat     = 0;
Theta_hat = [0; 0; 0];
x_ref     = [0; 0];
xcl0  = [phi_rad; p_rps; Kxhat; Krhat; Theta_hat; x_ref];

% Time domain
t0 = 0;
tf = 150;

% Reference model (2nd order transfer function)
wn_rps = 1; xi = 0.7;

A_ref = [0 1; -wn_rps^2 -2*xi*wn_rps];
B_ref = [0; wn_rps^2];

% Define adaptive rates
Q           = [1 0; 0 1];
Gamma_x     = 100*eye(2);
Gamma_r     = 100;
Gamma_theta = 100*eye(3);

% Ideal gains
Kx_ideal = -1/theta6*[wn_rps^2 + theta1, 2*xi*wn_rps + theta2];
Kr_ideal = wn_rps^2/theta6;

% Solve the Lyapunov equation AP + A'P = -Q for P
P = lyap(A_ref', Q);

% Integrate the adaptive controller
options = odeset('reltol', 1e-6, 'abstol', 1e-6);
[tcl, xcl] = ode45(@Closed_Loop_System_Ex_9p3, [t0 tf], xcl0, ...
    options, A, B, theta1, theta2, theta3, theta4, theta5,  ...
    theta6, P, Gamma_x, Gamma_r, Gamma_theta, A_ref, B_ref);

%% Part 2b: Plot the results of the adaptive controller
%--------------------------------------------------------------------------

% Calculate bank angle command
phi_cmd_deg = bank_angle_command(tcl);

figure(2)
subplot(2,1,1)
plot(tcl, phi_cmd_deg,'b-','linewidth',2); hold on 
plot(tcl, xcl(:,9)*180/pi,'g-','linewidth',2);
plot(tcl, xcl(:,1)*180/pi,'r-.','linewidth',2);
ylabel('Bank Angle [deg]','fontsize',14);
set(gca,'fontsize',14, 'xlim', [tcl(1) tcl(end)]);
h = legend('Command','Reference','Actual');
grid on

subplot(2,1,2)
plot(tcl, xcl(:,10)*180/pi,'b-','linewidth',2); hold on 
plot(tcl, xcl(:,2)*180/pi,'g-.','linewidth',2);
xlabel('Time [s]','fontsize',14);
ylabel('Roll Rate [deg/s]','fontsize',14);
set(gcf,'color','w');
set(gca,'fontsize',14, 'xlim', [tcl(1) tcl(end)]);
grid on 

% Plot adaptive gains
figure(3)
subplot(3,1,1)
plot(tcl, xcl(:,3), 'g-', 'linewidth', 2); hold on 
plot([tcl(1) tcl(end)], Kx_ideal(1)*[1 1], 'b-', 'linewidth', 2);
ylabel('K_{x1}','fontsize',14);
set(gca,'fontsize',14)
h = legend('Estimated','Ideal');
grid on 

subplot(3,1,2)
plot(tcl, xcl(:,4), 'g-', 'linewidth', 2); hold on 
plot([tcl(1) tcl(end)], Kx_ideal(2)*[1 1], 'b-', 'linewidth', 2);
ylabel('K_{x2}','fontsize',14);
set(gca,'fontsize',14);
grid on 

subplot(3,1,3)
plot(tcl, xcl(:,5), 'g-', 'linewidth', 2); hold on 
plot([tcl(1) tcl(end)], Kr_ideal*[1 1], 'b-', 'linewidth', 2);
xlabel('Time [s]','fontsize',14);
ylabel('K_{r}','fontsize',14);
set(gcf,'color','w');
set(gca,'fontsize',14, 'xlim', [tcl(1) tcl(end)]);
grid on 

% Plot adaptive parameters
figure
subplot(3,1,1)
plot(tcl, xcl(:,6), 'g-', 'linewidth', 2); hold on 
plot([tcl(1) tcl(end)], theta3*[1 1], 'b-', 'linewidth', 2);
ylabel('\theta_{3}','fontsize',14);
set(gca,'fontsize',14)
h = legend('Estimated','Ideal');
grid on 

subplot(3,1,2)
plot(tcl, xcl(:,7), 'g-', 'linewidth', 2); hold on 
plot([tcl(1) tcl(end)], theta4*[1 1], 'b-', 'linewidth', 2);
ylabel('\theta_{4}','fontsize',14);
set(gca,'fontsize',14);
grid on 

subplot(3,1,3)
plot(tcl, xcl(:,8), 'g-', 'linewidth', 2); hold on 
plot([tcl(1) tcl(end)], theta5*[1 1], 'b-', 'linewidth', 2);
xlabel('Time [s]','fontsize',14);
ylabel('\theta_{5}','fontsize',14);
set(gcf,'color','w');
set(gca,'fontsize',14, 'xlim', [tcl(1) tcl(end)]);
grid on 

function phi_cmd_deg = bank_angle_command(t)

phi_cmd_deg = zeros(size(t));

for i = 1 : length(t)

%     % step commands in text
%     if t(i)>=1 && t(i)<=10
%         phi_cmd_deg(i) = 5;
%     elseif t(i)>=22 && t(i) <=27
%         phi_cmd_deg(i) = -5;
%     elseif t(i)>=43 && t(i)<=49
%         phi_cmd_deg(i) = 10;
%     elseif t(i)>= 63 && t(i)<=73
%         phi_cmd_deg(i) = -10;
%     elseif t(i)>=85 && t(i)<= 95
%         phi_cmd_deg(i) = 5;
%     elseif t(i)>=105 && t(i)<=115
%         phi_cmd_deg(i) = -5;
%     end
    
%     phi_cmd_deg = 100*0.1745*sin(i);
    phi_cmd_deg = 2^i;
end
end

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
end 

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
end 