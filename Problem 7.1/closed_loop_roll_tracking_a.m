function dp = closed_loop_roll_tracking_a(t, p, Lp, Ld, a_ref, b_ref)

dp = zeros(2,1);

p_cmd = roll_rate_command(t);

% Direct control signal
dela = (a_ref - Lp)/Ld*p(1) + b_ref/Ld*p_cmd;

% Compute RHS of plant
dp(1,1) = Lp*p(1) + Ld*dela;

% Reference model
dp(2) = a_ref*p(2) + b_ref*p_cmd;