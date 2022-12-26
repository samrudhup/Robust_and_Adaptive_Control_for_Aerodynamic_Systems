% Problem 9.2 Lavretsky and Wise
%
% B. Dickinson 4/2/1014
%--------------------------------------------------------------------------

%% Problem 9.2

% Plant
Mq = -0.61;
Md = -6.65;

% Reference model
a_ref = -4;
b_ref =  4;

% Initial conditions
q0 = 0;
q_ref0 = 0;
kq0 = 0;
kq_cmd0 = 0;
theta0 = 0;

x0 = [q0, q_ref0, kq0, kq_cmd0, theta0];

% Time domain
t0 = 0;
tf = 75;

% Adaptive gain rates
% gamma_q = 1e-2;
% gamma_qcmd = 1e-2;
% Gamma_theta = 1e-2;
% 
gamma_q = 6e0;
gamma_qcmd = 6e0;
Gamma_theta = 8e0;
%
% gamma_q = 6e3;
% gamma_qcmd = 6e3;
% Gamma_theta = 8;

% Integrate the adaptive controller system
%--------------------------------------------------------------------------
options = odeset('reltol', 1e-3, 'abstol', 1e-3);
[t, x] = ode45(@Adaptive_System_Ex_9p2, [t0 tf], x0, options, Mq, ...
    Md, a_ref, b_ref, gamma_q, gamma_qcmd, Gamma_theta);

% Reproduce Figure 9.3 (The adaptive rates in the book do not give similar
% results)
%--------------------------------------------------------------------------
q_cmd = pitch_rate_command(t);

figure(1)
subplot(2,1,1)
plot(t, q_cmd,'b','linewidth', 2); hold on 
plot(t, x(:,2),'g-.','linewidth', 2);
plot(t, x(:,1),'r','linewidth', 2);
ylabel('q [deg/s]','fontsize',14);
set(gca,'fontsize',14, 'xlim', [t(1) t(end)]);
h = legend('Command','Reference','System');
grid on

subplot(2,1,2)
plot(t, x(:,3).*x(:,1) + x(:,4).*q_cmd' - x(:,5).*tanh(360/pi*x(:,1)),...
    'linewidth', 2);
xlabel('Time [s]','fontsize',14);
ylabel('\delta_e [rad/s]','fontsize',14);
set(gcf,'color','w');
set(gca,'fontsize',14, 'xlim', [t(1) t(end)]);
grid on 

% Reproduce figure 9.4 
%--------------------------------------------------------------------------

figure(2)
subplot(3,1,1)
plot(t, x(:,3), 'g', [t(1) t(end)], (a_ref - Mq)/Md*[1 1], '-.b', ...
    'linewidth', 2);
ylabel('k_q','fontsize',14);
set(gca,'fontsize',14, 'xlim', [t(1) t(end)]);
h = legend('Adaptive','Ideal');
grid on

subplot(3,1,2)
plot(t, x(:,4), 'g', [t(1) t(end)], b_ref/Md*[1 1], '-.b', ...
    'linewidth', 2);
ylabel('k_{q,cmd}','fontsize',14);
set(gcf,'color','w');
set(gca,'fontsize',14, 'xlim', [t(1) t(end)]);
grid on 

subplot(3,1,3)
plot(t, x(:,5), 'g', [t(1) t(end)], -0.01*[1 1], '-.b', ...
    'linewidth', 2);
xlabel('Time [s]','fontsize',14);
ylabel('\theta','fontsize',14);
set(gcf,'color','w');
set(gca,'fontsize',14, 'xlim', [t(1) t(end)]);
grid on

% Reproduce figure 9.5
%--------------------------------------------------------------------------
q_cmd = pitch_rate_command(t);

figure(3)
subplot(2,1,1)
plot(t, q_cmd,'b','linewidth', 2); hold on 
plot(t, x(:,2),'g-.','linewidth', 2);
plot(t, x(:,1),'r','linewidth', 2);
ylabel('q [deg/s]','fontsize',14);
set(gca,'fontsize',14, 'xlim', [t(1) t(end)], 'ylim', [-3 3]);
h = legend('Command','Reference','System');
grid on

subplot(2,1,2)
plot(t, (x(:,3).*x(:,1) + x(:,4).*q_cmd' - x(:,5).*tanh(360/pi*x(:,1)))*180/pi,...
    'linewidth', 2);
xlabel('Time [s]','fontsize',14);
ylabel('\delta_e [deg]','fontsize',14);
set(gcf,'color','w');
set(gca,'fontsize',14, 'xlim', [t(1) t(end)]);
grid on 

