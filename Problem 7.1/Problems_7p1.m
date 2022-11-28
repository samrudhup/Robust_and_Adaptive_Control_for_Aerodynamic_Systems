% Problems 7.1 and 7.3 Lavretsky and Wise

%% Problem 7.1

% Plant
Lp = -0.8;
Ld =  1.6;

% Reference model
a_ref = -2;
b_ref =  2;

% Initial conditions
p0 = 1;
p_ref = 0;

% Time domain
t0 = 0;
tf = 5;

%% Part 1 - Investigate command tracking ability of reference system
% (baseline controller design, the "ideal" response)
%--------------------------------------------------------------------------
close all 

options = odeset('reltol', 1e-6, 'abstol', 1e-6);
[t1, pref] = ode45(@closed_loop_roll_tracking_reference, [t0 tf], p0, ...
    options, a_ref, b_ref);

p_cmd = roll_rate_command(t1);

figure(2)
plot(t1, p_cmd, 'b', t1, pref, 'g', 'linewidth', 2); hold on
xlabel('time [s]','fontsize',14);
ylabel('Roll Rate [rad/s]','fontsize',14);
set(gca,'fontsize',14,'xlim',[t0 tf]);
set(gcf,'color','w');
h = legend('Roll Rate Command','Model Reference Roll Rate');
set(h, 'location', 'best')
grid on 

%% Part 1A - Design a fixed gain model reference controller to recover the
% reference model dynamics. The control should have the form of 7.2.
%--------------------------------------------------------------------------
close all

options = odeset('reltol', 1e-6, 'abstol', 1e-6);
[t1, p] = ode45(@closed_loop_roll_tracking_a, [t0 tf], [p0; p_ref], ...
    options, Lp, Ld, a_ref, b_ref);

p_cmd = roll_rate_command(t1);

sel_p = 1;
sel_pref = 2;

figure(3)
plot(t1, p_cmd, 'b', t1, p(:,sel_p), 'r', t1, p(:,sel_pref),'g',...
    'linewidth', 2); hold on
xlabel('time [s]','fontsize',14);
ylabel('Roll Rate [rad/s]','fontsize',14);
set(gca,'fontsize',14,'xlim',[t0 tf]);
set(gcf,'color','w');
h = legend('Roll Rate Command','Closed Loop Roll Rate','Model Reference Roll Rate');
set(h, 'location', 'best')
grid on 

%% Part 1B - Design a fixed gain model reference controller to recover the
% reference model dynamics. The control should have the form of 7.9
%--------------------------------------------------------------------------
close all

% Error fixed gain
ke = -10;

[t2, p2] = ode45(@closed_loop_roll_tracking_b, [t0 tf], [p0; p_ref], ...
    options, Lp, Ld, a_ref, b_ref, ke);

p_cmd = roll_rate_command(t1);

sel_p = 1;
sel_pref = 2;

figure(2)
plot(t1, p_cmd, 'b', t2, p2(:,sel_p), 'r', t2, p2(:,sel_pref),'g',...
    'linewidth',2); hold on
xlabel('time [s]','fontsize',14);
ylabel('Roll Rate [rad/s]','fontsize',14);
set(gca,'fontsize',14,'xlim',[t0 7]);
set(gcf,'color','w');
h = legend('Roll Rate Command','Closed Loop Roll Rate w/error Gain','Model Reference Roll Rate');
set(h, 'location', 'best')
grid on 


