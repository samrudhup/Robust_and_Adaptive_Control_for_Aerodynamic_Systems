%% Problem 7.3

% Time domain
t0 = 0;
tf = 15;

% Error fixed gain
ke = -10;

% Adaptive gain rates
gammap = 1e4;
gammap_cmd = 1e4;

% Ideal adaptive gains
kp_ideal = (a_ref - Lp)/Ld;
kpcmd_ideal = b_ref/Ld;

% Initial adaptive gains
kp = 0;
kp_cmd = 0;

% Part 2A - Design an adaptive model reference controller 
%--------------------------------------------------------------------------
close all

[t3, x] = ode45(@closed_loop_roll_tracking_c, [t0 tf], ...
    [p0, p_ref, kp, kp_cmd]', options, Lp, Ld, a_ref, b_ref, gammap, ...
    gammap_cmd, ke);

% Part 2B - Plot the adaptive simulation and compare to the direct fixed
% gain control law
%--------------------------------------------------------------------------
sel_p_plant   = 1;
sel_p_ref     = 2;
sel_kp_hat    = 3;
sel_kpcmd_hat = 4;

% Plot the roll rate command and the reference model state
figure
p_cmd = roll_rate_command(t3);
plot(t3, p_cmd, 'b', t3, x(:,sel_p_plant), 'r', t3, x(:,sel_p_ref), ...
    'g', 'linewidth', 2); hold on
xlabel('time [s]','fontsize',14);
ylabel('Roll Rate [rad/s]','fontsize',14);
title('Direct MRAC Roll Rate Control','fontsize',14);
set(gca,'fontsize',14,'xlim',[t0 tf]);
set(gcf,'color','w');
h = legend('Roll Rate Command','Plant Roll Rate', ...
    'Reference Roll Rate');
set(h, 'location', 'best')
grid on 

figure
plot(t3,  x(:,sel_p_plant)- x(:,sel_p_ref),'k', 'linewidth', 2); hold on
xlabel('time [s]','fontsize',14);
ylabel('Roll Rate Error [rad/s]','fontsize',14);
% title('Roll Rate Error Evoluation','fontsize',14);
set(gca,'fontsize',14,'xlim',[t0 tf]);
set(gcf,'color','w');
h = legend('e(t) = p(t)-p_{ref}(t)');
set(h, 'location', 'best')
grid on 

figure
subplot(2,1,1)
plot(t3, x(:, sel_kp_hat), 'b', 'linewidth', 2); hold on 
plot([t3(1), t3(end)], kp_ideal*[1 1], 'r', 'linewidth', 2);
h = legend('\^{k_p}','k_p');
set(h,'fontsize',14, 'location', 'best');
set(gca, 'fontsize', 14, 'xlim', [t0 tf]);
title('Adaptive Gain Estimates','fontsize',14);
grid on 

subplot(2,1,2)
plot(t3, x(:,4), 'b', 'linewidth', 2); hold on 
plot([t3(1), t3(end)], kpcmd_ideal*[1 1], 'r', 'linewidth', 2);
xlabel('Time [s]','fontsize',14);
h = legend('\^{k_p_{cmd}}','k_p_{cmd}');
set(h,'fontsize',14, 'location', 'best');
set(gca, 'fontsize', 14, 'xlim', [t0 tf]);
set(gcf, 'color', 'w');
grid on 

