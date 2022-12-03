% Problem 2.4 Wise and Lavretsky with additional parts a-d
%--------------------------------------------------------------------------
clear all 
close all
clc
% Problem 2.4
%--------------------------------------------------------------------------
% State indices
sel_V_state_fps     = 1;
sel_alpha_state_rad = 2;
sel_theta_state_rad = 3;
sel_q_state_rps     = 4;
% Input indices
sel_dele_input_rad = 1;
sel_delth_input    = 2;
% The open loop plant
Ap = [ -0.0160  18.91 -32.2      0; ...
        -0.001 -0.633     0      1; ...
             0      0     0      1; ...
             0 -0.773     0 -0.528]; 
nAp = size(Ap,1);
           
Bp = [      0  9.968; ... 
        0.010 -0.006; ... 
            0      0; ... 
       -0.011 0.0256];
mBp = size(Bp,2);
         
Cp = eye(nAp);
% Define state space object
sys_plant = ss(Ap, Bp, Cp, 0);
% Put labels on inputs, outputs, and states
set(sys_plant, 'StateName',  {'V [ft/s]','alpha [rad]', 'theta [rad]','q [r/s]'}, ...'OutputName', {'V [ft/s]','alpha [rad]', 'theta [rad]','q [r/s]'}, ...
               'InputName',  {'dele [rad]','deleth [unitless]'});
           
% State feedback penalties
Q = 1e3*diag([0.01 1 1 1]);
R = diag([.01 1]);
% Check controllability and observability requirements
CtrlMtx = [Bp, Ap*Bp, Ap*Ap*Bp, Ap*Ap*Ap*Bp]; 
if rank(CtrlMtx) < nAp
    disp('Open loop system is not controllable');
    t_final
else
    disp('Open loop system is controllable.');
end
Q_sqrt = sqrt(Q);
ObsvMtx = [Q_sqrt'; Ap'*Q_sqrt'; Ap'*Ap'*Q_sqrt'; Ap'*Ap'*Ap'*Q_sqrt'];
if rank(ObsvMtx) < nAp
        disp('The pair (A,Q^1/2) is not observable');
    t_final
else
    disp('The pair (A,Q^1/2) is observable.');
end
           
% Compute LQR state feedback gains
[Kc, ~, ~] = lqr(Ap, Bp, Q, R);
% Build closed loop system
Acl = Ap - Bp*Kc;
% Check properties of the closed loop system (no design specifications were
% made, here we're just verifying that the system is stable and that it's
% not too underdamped, roughly damping >0.5)
fprintf('Closed Loop Eigenvalues:\n')
damp(Acl);
% Build the state space closed loop system
sys_lqr_cl = ss(Acl, Bp*0, Cp, 0);
set(sys_lqr_cl, 'StateName',  {'V [ft/s]', 'alpha [rad]', 'theta [rad]', 'q [r/s]'},'OutputName', {'V [ft/s]', 'alpha [rad]', 'theta [rad]', 'q [r/s]'},'InputName',  {'dele [rad]', 'deleth [unitless]'});
% Initial condition specified in the textbook
x_cl0 = [10, 0.1, 0, 0.1];
% Define time step size
dt = 1e-2;
t0 = 0;
t_final = 10;
t_sec = t0 : dt : t_final; nt_sec = length(t_sec);
            
% preallocate open-loop solution vector due to elevator input
x_cl = zeros(nAp, nt_sec);
x_cl(:,1) = x_cl0;
% Use trapezoidal rule to integrate closed loop system, 2nd order accurate
for i = 1 : length(t_sec) - 1
    
    % Closed loop regulation
    x_cl(:, i+1)   = x_cl(:,i) + dt*( Acl*x_cl(:,i) );
    x_cl(:, i+1)   = x_cl(:,i) + dt/2*( Acl*x_cl(:,i) + Acl*x_cl(:,i+1) );
    
end
% Plot closed loop states
figure(1)
title('Closed loop system','fontsize', 14);
subplot(4,1,1);
plot(t_sec, x_cl(sel_V_state_fps,:), 'linewidth', 2); grid on
ylabel({'V_T', '[ft/s]'}, 'fontsize', 18);
set(gca, 'xlim', [0 t_final]);
set(gca, 'fontsize', 18);
subplot(4,1,2);
plot(t_sec, x_cl(sel_alpha_state_rad,:), 'linewidth', 2); grid on
ylabel({'\alpha', '[rad]'}, 'fontsize', 18)
set(gca, 'fontsize', 18);
set(gca, 'xlim', [0 t_final]);
subplot(4,1,3);
plot(t_sec, x_cl(sel_q_state_rps,:), 'linewidth', 2); grid on
ylabel({'q', '[rad/s]'}, 'fontsize', 18);
set(gca, 'fontsize', 18);
set(gca, 'xlim', [0 t_final]);
subplot(4,1,4);
plot(t_sec, x_cl(sel_theta_state_rad,:), 'linewidth', 2); grid on
ylabel({'\theta', '[rad]'}, 'fontsize', 18);
xlabel('Time [s]','fontsize', 18);
set(gca, 'fontsize', 18);
set(gca, 'xlim', [0 t_final]);
set(gcf, 'color', 'w');

% Plot closed loop control 
u = -Kc*x_cl;
figure(2)
title('Closed loop system','fontsize', 14);
subplot(2,1,1);
plot(t_sec, u(2,:), 'linewidth', 2); grid on
ylabel('\delta_{th}', 'fontsize', 14);
set(gca, 'xlim', [0 t_final]);
set(gca, 'fontsize', 18);
subplot(2,1,2);
plot(t_sec, u(1,:), 'linewidth', 2); grid on
ylabel({'\delta_e', '[rad]'}, 'fontsize', 18);
set(gca, 'fontsize', 18);
set(gca, 'xlim', [0 t_final]);
set(gcf, 'color', 'w');
%% a) Extract the short period
%--------------------------------------------------------------------------
Asp = Ap([sel_alpha_state_rad, sel_q_state_rps], ...
         [sel_alpha_state_rad, sel_q_state_rps]); nAsp = size(Asp,1);
         
Bsp = Bp([sel_alpha_state_rad; sel_q_state_rps]);
sel_alpha_spstate_rad = 1;
sel_q_spstate_rps = 2;
sel_dele_spinput_rad = 3;
%% a) Compare the closed loop response with the open loop response
%--------------------------------------------------------------------------
% Set penalties
Qsp = Q([sel_alpha_state_rad, sel_q_state_rps], ...
        [sel_alpha_state_rad, sel_q_state_rps]);
Rsp = R(sel_dele_input_rad, sel_dele_input_rad);
% LQR Gains
Ksp = lqr(Asp, Bsp, Qsp, Rsp);
% Closed loop short period
Aspcl = (Asp - Bsp*Ksp);
% Preallocate open-loop solution vector
x_ol_dele = zeros(size(Asp,2),length(t_sec));
x_cl_dele = x_ol_dele;
% Set initial conditions
xsp_cl0 = x_cl0([sel_alpha_state_rad, sel_q_state_rps]);
x_ol_dele(:,1) = xsp_cl0;
x_cl_dele(:,1) = xsp_cl0;
% define zero input (only looking at zero input response)
u = zeros(size(t_sec));
for i = 1 : length(t_sec) - 1
    
    % Open loop response due to elevator input  
    x_ol_dele(:, i+1)   = x_ol_dele(:,i) + ...
        dt/2*( Asp*x_ol_dele(:,i)+Bsp*u(i) + ...
               Asp*x_ol_dele(:,i+1)+Bsp*u(i+1));
    
    % Closed loop response due to elevator input
    x_cl_dele(:, i+1)   = x_cl_dele(:,i) + ...
        dt/2*( Aspcl*x_cl_dele(:,i) + ...
               Aspcl*x_cl_dele(:,i+1));
    
end
figure(3)
plot(t_sec, x_ol_dele(sel_alpha_spstate_rad,:),'k--','linewidth', 2); hold on 
plot(t_sec, x_cl_dele(sel_alpha_spstate_rad,:),'r', 'linewidth', 2); 
ylabel({'\alpha', '[rad]'}, 'fontsize', 18);
xlabel('Time [s]','fontsize', 18);
set(gca, 'xlim', [0 t_final]);
set(gca, 'fontsize', 18);
set(gcf, 'color', 'w');
grid on
h = legend('Open Loop','Closed Loop');
set(h,'location','best')
figure(4)
plot(t_sec, x_ol_dele(sel_q_spstate_rps,:),'k--','linewidth', 2); hold on 
plot(t_sec, x_cl_dele(sel_q_spstate_rps,:),'r', 'linewidth', 2); 
ylabel({'q', '[r/s]'}, 'fontsize', 18);
xlabel('Time [s]','fontsize', 18);
set(gca, 'xlim', [0 t_final]);
set(gca, 'fontsize', 18);
set(gcf, 'color', 'w');
grid on
h = legend('Open Loop','Closed Loop');
set(h,'location','best')
%% c) Let Q = [1 1; 1 1], and R = 1/rho (as in the lectures) where rho is 
% a positive real number. Compute the closed loop eigenvalues for a 
% sequence of increasing values of rho.
%
% e) Include the actuator in the loop and repeat
%--------------------------------------------------------------------------
% Extract short period from coupled longitudinal dynamic equations
Csp = eye(nAsp);
Dsp = 0*Csp*Bsp;
% Get open loop eigenvalues
eig_ol = eig(Asp);
% Build state space system
sys_plant_sp = ss(Asp, Bsp, Csp, Dsp);
% Put labels on inputs, outputs, and states
set(sys_plant_sp, 'StateName',  {'alpha [rad]', 'q [r/s]'}, ...
                  'OutputName', {'alpha [rad]', 'q [r/s]'}, ...
                  'InputName',  {'dele [rad]'});
% Second order actuator model
wa = 2*pi*30;  % Natural frequency, rad/s
za = 0.70;     % Damping ratio    , n/d
Ap_act = [0, 1; -wa^2, -2*za*wa];  nAp_act = size(Ap_act,1);
Bp_act = [0;  wa^2];       
Cp_act = [1, 0];
Dp_act = 0;
              
% rho parameter
nrho = 100;
rho_param = logspace(0, 12, nrho);
% Define base penalty matrices
Q = ones(nAsp, nAsp);
% Preallocate eigenvalue array
eig_cl = zeros(nAsp, nrho);
eig_cl_act = zeros(nAsp + nAp_act, nrho);
for ii = 1 : nrho
    
    % Check controllability and observability requirements
    CtrlMtx = [Bsp, Asp*Bsp]; 
    if rank(CtrlMtx) < nAsp
        disp('Open loop system is not controllable');
        t_final
    else
        disp('Open loop system is controllable.');
    end
    Q_sqrt = 1/sqrt(2)*Q;
    ObsvMtx = [Q_sqrt'; Asp'*Q_sqrt'];
    if rank(ObsvMtx) < nAsp    
        disp('The pair (A,Q^1/2) is not observable');
        % Check detectability
        if real(eig_ol) >= 0
            disp('The open loop system contains unstable modes. Detectability must be checked.');
        else
            disp('Open loop stable system. Detectability ensured.');
        end
    else
        disp('The pair (A,Q^1/2) is observable.');
    end
    % Solve ARE, compute gains (note design model does not include actuator
    % model)
    [Kc, Pw, ~] = lqr(Asp, Bsp, Q, 1/rho_param(ii));
    % Get the closed-loop eigenvalues
    eig_cl(:,ii) = eig(Asp - Bsp*Kc);
    
    % Build the closed loop system with the actuator in the loop
    Ap_sp_act = [Asp, Bsp*Cp_act; -Bp_act*Kc, Ap_act];
    
    % Get the closed-loop eigenvalues with the actuator in the loop
    eig_cl_act(:,ii) = eig(Ap_sp_act);
    
end
% Get zeros of d^T*(sI-A)^-1*B_sp for the longitudinal dynamics only
d = [1;1];
H = ss(Asp, Bsp, d', 0*d'*Bsp);
Htf = tf(H);
H_zero = -20.41;
% c. Plot the locus of closed loop eigenvalues and explain their trajectories
%--------------------------------------------------------------------------
figure(7)
plot(real(eig_cl),imag(eig_cl), 'k*','markersize',10); hold on 
plot(real(eig_ol),imag(eig_ol), 'og','markersize',10, 'linewidth', 2); 
plot(real(H_zero),imag(H_zero),'sr','markersize',10,'linewidth', 2); 
title('Closed Loop Root Locus (no actuator)','fontsize', 18);
ylabel('Im', 'fontsize', 18);
xlabel('Re','fontsize', 18);
set(gca, 'fontsize', 18, 'xlim', [-50 0], 'ylim', [-15 15]);
set(gcf, 'color', 'w');
grid on
axis equal
% c. Plot the locus of closed loop eigenvalues and explain their trajectories
%--------------------------------------------------------------------------
figure(8)
plot(real(eig_cl_act),imag(eig_cl_act), '*k','markersize',10); hold on 
%plot(real(eig_cl),imag(eig_cl), '*y','markersize',10); 
plot(real(eig_ol),imag(eig_ol), 'og','markersize',10, 'linewidth', 2); 
plot(real(H_zero),imag(H_zero),'sr','markersize',10,'linewidth', 2); 
title('Closed Loop Root Locus w Actuator','fontsize', 18);
ylabel('Im', 'fontsize', 18);
xlabel('Re','fontsize', 18);
set(gca, 'fontsize', 18);
set(gcf, 'color', 'w');
grid on
axis equal
figure(9)
plot(real(eig_cl_act),imag(eig_cl_act), '*k','markersize',10); hold on 
%plot(real(eig_cl),imag(eig_cl), '*y','markersize',10); 
plot(real(eig_ol),imag(eig_ol), 'og','markersize',10, 'linewidth', 2); 
plot(real(H_zero),imag(H_zero),'sr','markersize',10,'linewidth', 2); 
title('Closed Loop Root Locus w Actuator','fontsize', 18);
ylabel('Im', 'fontsize', 18);
xlabel('Re','fontsize', 18);
set(gca, 'fontsize', 18, 'xlim', [-50 0], 'ylim', [-15 15]);
set(gcf, 'color', 'w');
grid on
axis equal