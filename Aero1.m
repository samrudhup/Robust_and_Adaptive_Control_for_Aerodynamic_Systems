% Problem 1.2 Wise and Lavretsky

% State indices
sel_V_state_fps     = 1;
sel_alpha_state_rad = 2;
sel_theta_state_rad = 3;
sel_q_state_rps     = 4;
% Input indices
sel_dele_input_rad  = 1;
sel_delth_input_rad = 2;
% A and B matrices
A_long = [-0.160  18.91 -32.2      0; ...
          -0.001 -0.633     0      1; ...
               0      0     0      1; ...
               0 -0.773     0 -0.528];
    
B_long = [     0  9.968; ...
               0 -0.006; ...
               0      0; ...
          -0.011 0.0256];
% Trimmed speed
V0_fps = 250;
% a) Compute open loop system eigenvalues
[wn_long, xi_long, long_eigs] = damp(A_long);
% b) Extract short period dynamics, compute approximate short
% period 
A_sp = A_long([sel_alpha_state_rad, sel_q_state_rps], ...
              [sel_alpha_state_rad, sel_q_state_rps]);
B_sp = B_long([sel_alpha_state_rad, sel_q_state_rps], sel_dele_input_rad);
[wn_sp, xi_sp, sp_eigs] = damp(A_sp);
% c) Simulate open loop response to elevator and thrust set inputs
dt = 1e-2;
tfinal_sec = 500;
t_sec = 0 : dt : tfinal_sec;
% Initial condition 
x_cl0 = [0 0 0 0];
% preallocate open-loop solution vector due to thrust input
x_ol_thrust = zeros(size(A_long,2),length(t_sec));
x_ol_thrust(:,1) = x_cl0;
% preallocate open-loop solution vector due to elevator input
x_ol_dele = zeros(size(A_long,2),length(t_sec));
x_ol_dele(:,1) = x_cl0;
% define step input at 1 sec
u = zeros(size(t_sec));
u(t_sec >= 50) = -1;
for i = 1 : length(t_sec) - 1
    
% Open loop response due to thrust input
% ##    x_ol_dele(:, i+1)   = x_ol_dele(:,i) + ...
% ##        dt*( A_long*x_ol_dele(:,i)+B_long*[u(i);0]);
%     
    x_ol_dele(:, i+1)   = x_ol_dele(:,i) + ...
        dt/2*( A_long*x_ol_dele(:,i)+B_long*[u(i);0] + ...
               A_long*x_ol_dele(:,i+1)+B_long*[u(i+1);0]);
% Open loop response due to thrust input
% ##    x_ol_thrust(:, i+1) = x_ol_thrust(:,i) + ...
% ##        dt*( A_long*x_ol_thrust(:,i)+B_long*[0; u(i)]);
%     
    x_ol_thrust(:, i+1) = x_ol_thrust(:,i) + ...
        dt/2*( A_long*x_ol_thrust(:,i)+B_long*[0; u(i)] + ...
               A_long*x_ol_thrust(:,i+1)+B_long*[0; u(i+1)]);
    
end
figure(1)
title('Open-Loop Response Elevator Step Input','fontsize', 14);
subplot(2,2,1);
plot(t_sec, x_ol_dele(sel_V_state_fps,:), 'linewidth', 2); grid on
ylabel('Air Speed [ft/s]', 'fontsize', 16);
xlabel('Time [s]','fontsize', 16);
set(gca, 'xlim', [0 300]);
set(gca, 'fontsize', 14);
subplot(2,2,2);
plot(t_sec, x_ol_dele(sel_alpha_state_rad,:), 'linewidth', 2); grid on
ylabel('AoA [rad]', 'fontsize', 16);
xlabel('Time [s]','fontsize', 16);
set(gca, 'fontsize', 14);
set(gca, 'xlim', [0 300]);
subplot(2,2,3);
plot(t_sec, x_ol_dele(sel_q_state_rps,:), 'linewidth', 2); grid on
ylabel('Pitch Rate [rad/s]', 'fontsize', 16);
xlabel('Time [s]','fontsize', 16);
set(gca, 'fontsize', 14);
set(gca, 'xlim', [0 300]);
subplot(2,2,4);
plot(t_sec, x_ol_dele(sel_theta_state_rad,:), 'linewidth', 2); grid on
ylabel('Pitch Angle [rad]', 'fontsize', 16);
xlabel('Time [s]','fontsize', 16);
set(gca, 'fontsize', 14);
set(gca, 'xlim', [0 300]);
set(gcf, 'color', 'w');
figure(2)
title('Open-Loop Response Thrust Step Input','fontsize', 16);
subplot(2,2,1);
plot(t_sec, x_ol_thrust(sel_V_state_fps,:), 'linewidth', 2); grid on
ylabel('Air Speed [ft/s]', 'fontsize', 16);
xlabel('Time [s]','fontsize', 16);
set(gca, 'xlim', [0 300]);
set(gca, 'fontsize', 14);
subplot(2,2,2);
plot(t_sec, x_ol_thrust(sel_alpha_state_rad,:), 'linewidth', 2); grid on
ylabel('AoA [rad]', 'fontsize', 16);
xlabel('Time [s]','fontsize', 16);
set(gca, 'fontsize', 14);
set(gca, 'xlim', [0 300]);
subplot(2,2,3);
plot(t_sec, x_ol_thrust(sel_q_state_rps,:), 'linewidth', 2); grid on
ylabel('Pitch Rate [rad/s]', 'fontsize', 16);
xlabel('Time [s]','fontsize', 16);
set(gca, 'fontsize', 14);
set(gca, 'xlim', [0 300]);
subplot(2,2,4);
plot(t_sec, x_ol_thrust(sel_theta_state_rad,:), 'linewidth', 2); grid on
ylabel('Pitch Angle [rad]', 'fontsize', 16);
xlabel('Time [s]','fontsize', 16);
set(gca, 'fontsize', 14);
set(gca, 'xlim', [0 300]);
set(gcf, 'color', 'w');
% Identify frequency time scale separation between short period and phugoid
% wn_phugoid_rad    = sqrt(real(long_eigs(1))^2   + imag(long_eigs(1))^2);
% wn_sp_rad         = sqrt(real(long_eigs(end))^2 + imag(long_eigs(end))^2);
% 
% fn_phugoid = wn_phugoid_rad/(2*pi);
% fn_sp      = wn_sp_rad/(2*pi);
% 
% T_sp_sec = 1/fn_phugoid;
% T_phugoid_sec = 1/fn_sp;
% Time scale separation between short period and phugoid modes
% time_scale_separation_frequency = T_phugoid_sec-T_sp_sec;
% fprintf('The time scale separation between the frequency of the short period and phugoid is %.1f sec\n', ...
%     time_scale_separation_frequency);
% Time scale separation between decay/amplification of modes
real_part_sp_eig = real(long_eigs(1));
real_part_ph_eig = real(long_eigs(3));
t_star_sp_sec = abs(1/real_part_sp_eig);
t_star_ph_sec = abs(1/real_part_ph_eig);
time_scale_separation_decay = t_star_ph_sec - t_star_sp_sec;
fprintf('The time scale separation between the short period and phugoid is %.1f sec\n',time_scale_separation_decay);
% Determine aero derivatives
Zalpha = A_long(sel_alpha_state_rad, sel_alpha_state_rad)*V0_fps;
Malpha = A_long(sel_q_state_rps, sel_alpha_state_rad);
Zdelta = B_long(sel_alpha_state_rad, sel_dele_input_rad)*V0_fps;
Mdelta = B_long(sel_q_state_rps, sel_dele_input_rad);
Mq     = A_long(sel_q_state_rps, sel_q_state_rps);
% Plot vertical acceleration
Az = Zalpha*x_ol_dele(sel_alpha_state_rad,:) + Zdelta*u;
figure(3)
plot(t_sec, Az, 'linewidth', 2)
xlabel('Time [s]','fontsize', 16);
ylabel('A_Z [ft/s^2]','fontsize', 16);
set(gca, 'fontsize', 16, 'xlim', [0 200]);
set(gcf, 'color', 'w');
grid on;