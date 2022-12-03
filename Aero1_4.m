% Problem 1.4, Wise and Lavresky
clear all 
close all
clc
% State indices
sel_phi_state_rad    = 1;
sel_beta_state_rad = 2;
sel_p_state_rps     = 3;
sel_r_state_rps = 4;
% Input indices
sel_dela_input_rad = 1;
sel_delr_input_rad = 2;
% Lateral directional matrices
A_lat = [0 0 1 0; 0.0487 -0.0829 0 -1; 0 -4.546 -1.699 0.1717; ...
    0 3.382 -0.0654 -0.0893];
B_lat = [0 0; 0 0.0116; 27.276 0.5758; 0.3952 -1.362];
% Extract dutch roll and roll rate dynamics from A_lat and B_lat
A_lat_2 = A_lat([sel_beta_state_rad, sel_p_state_rps, sel_r_state_rps], ...
    [sel_beta_state_rad, sel_p_state_rps, sel_r_state_rps]);
B_lat_2 = B_lat([sel_beta_state_rad, sel_p_state_rps, sel_r_state_rps], :);
% Trimmed speed
V0_fps = 250;
% a) Compute open loop system eigenvalues
disp('The open loop eigenvalues are');
lat_eigs = eig(A_lat)
% b) Simulate open loop response to aileron and rudder set inputs
dt = 1e-3;
t_sec = 0 : dt : 50;
% Initial condition 
x_cl0 = [0 0 0 0];
x_cl0_2 = [0 0 0];
% preallocate open-loop solution vector due to thrust input
x_ol_aileron = zeros(size(A_lat,2),length(t_sec));
x_ol_aileron(:,1) = x_cl0;
% preallocate open-loop solution vector due to elevator input
x_ol_rudder = zeros(size(A_lat,2), length(t_sec));
x_ol_rudder(:,1) = x_cl0;
% preallocate open-loop solution vector due to thrust input for reduced
% system
x_ol_aileron_2 = zeros(size(A_lat_2,2),length(t_sec));
x_ol_aileron_2(:,1) = x_cl0_2;
% preallocate open-loop solution vector due to elevator input for reduced
% system
x_ol_rudder_2 = zeros(size(A_lat_2,2), length(t_sec));
x_ol_rudder_2(:,1) = x_cl0_2;
% define step input at 1 sec
u = zeros(size(t_sec));
u(t_sec>=1) = 1;
for i = 1 : length(t_sec) - 1
    
    % Open loop response due to rudder input
    x_ol_rudder(:, i+1)   = x_ol_rudder(:,i) + ...
        dt*(A_lat*x_ol_rudder(:,i)+B_lat*[0; u(i)]);
    % Open loop response due to aileron input
    x_ol_aileron(:, i+1) = x_ol_aileron(:,i) + ...
        dt*(A_lat*x_ol_aileron(:,i)+B_lat*[u(i); 0]);
    
    % Open loop response due to rudder input
    x_ol_rudder_2(:, i+1)   = x_ol_rudder_2(:,i) + ...
        dt*(A_lat_2*x_ol_rudder_2(:,i)+B_lat_2*[0; u(i)]);
    % Open loop response due to aileron input
    x_ol_aileron_2(:, i+1) = x_ol_aileron_2(:,i) + ...
        dt*(A_lat_2*x_ol_aileron_2(:,i)+B_lat_2*[u(i); 0]);
    
end
figure(1)
title('Open-Loop Response Elevator Step Input','fontsize', 14);
subplot(2,2,1);
plot(t_sec, x_ol_rudder(1,:), 'linewidth', 2); grid on
ylabel('Roll Angle [rad]', 'fontsize', 14);
xlabel('Time [s]','fontsize', 14);
set(gca, 'xlim', [0 50]);
set(gca, 'fontsize', 14);
subplot(2,2,2);
plot(t_sec, x_ol_rudder(2,:), 'linewidth', 2); grid on
ylabel('Side Slip [rad]', 'fontsize', 14);
xlabel('Time [s]','fontsize', 14);
set(gca, 'fontsize', 14);
set(gca, 'xlim', [0 50]);
subplot(2,2,3);
plot(t_sec, x_ol_rudder(3,:), 'linewidth', 2); grid on
ylabel('Roll Rate [rad/s]', 'fontsize', 14);
xlabel('Time [s]','fontsize', 14);
set(gca, 'fontsize', 14);
set(gca, 'xlim', [0 50]);
subplot(2,2,4);
plot(t_sec, x_ol_rudder(4,:), 'linewidth', 2); grid on
ylabel('Yaw Rate [rad/s]', 'fontsize', 14);
xlabel('Time [s]','fontsize', 14);
set(gca, 'fontsize', 14);
set(gca, 'xlim', [0 50]);
set(gcf, 'color', 'w');
figure(2)
title('Open-Loop Response Elevator Step Input','fontsize', 14);
subplot(2,2,1);
plot(t_sec, x_ol_aileron(1,:), 'linewidth', 2); grid on
ylabel('Roll Angle [rad]', 'fontsize', 14);
xlabel('Time [s]','fontsize', 14);
set(gca, 'xlim', [0 50]);
set(gca, 'fontsize', 14);
subplot(2,2,2);
plot(t_sec, x_ol_aileron(2,:), 'linewidth', 2); grid on
ylabel('Side Slip [rad]', 'fontsize', 14);
xlabel('Time [s]','fontsize', 14);
set(gca, 'fontsize', 14);
set(gca, 'xlim', [0 50]);
subplot(2,2,3);
plot(t_sec, x_ol_aileron(3,:), 'linewidth', 2); grid on
ylabel('Roll Rate [rad/s]', 'fontsize', 14);
xlabel('Time [s]','fontsize', 14);
set(gca, 'fontsize', 14);
set(gca, 'xlim', [0 50]);
subplot(2,2,4);
plot(t_sec, x_ol_aileron(4,:), 'linewidth', 2); grid on
ylabel('Yaw Rate [rad/s]', 'fontsize', 14);
xlabel('Time [s]','fontsize', 14);
set(gca, 'fontsize', 14);
set(gca, 'xlim', [0 50]);
set(gcf, 'color', 'w');
figure(3)
plot(t_sec, x_ol_rudder_2, '--', ...
     t_sec, x_ol_rudder(2,:), ...
     t_sec, x_ol_rudder(3,:), t_sec, x_ol_rudder(4,:), 'linewidth', 2)
xlabel('Time [s]','fontsize', 14);
title('Open-Loop Response Rudder Step Input (Reduced System)','fontsize', 14);
set(gca, 'fontsize', 14);
set(gcf, 'color', 'w');
h = legend('beta approx [rad]', 'p approx [rad/s]', 'r approx [rad/s]', 'beta [rad]', 'p [rad/s]', 'r [rad/s]');
set(h, 'fontsize', 14);
figure(4)
plot(t_sec, x_ol_aileron_2, '--', ...
     t_sec, x_ol_aileron(2,:), ...
     t_sec, x_ol_aileron(3,:), t_sec, x_ol_aileron(4,:), 'linewidth', 2)
xlabel('Time [s]','fontsize', 14);
title('Open-Loop Response Aileron Step Input (Reduced System)','fontsize', 14);
set(gca, 'fontsize', 14);
set(gcf, 'color', 'w');
h = legend('beta approx [rad]', 'p approx [rad/s]', 'r approx [rad/s]', 'beta [rad]', 'p [rad/s]', 'r [rad/s]');
set(h, 'fontsize', 14);