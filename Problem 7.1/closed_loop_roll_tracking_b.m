function dp = closed_loop_roll_tracking_b(t, p, Lp, Ld, a_ref, b_ref, ke)

p_cmd = roll_rate_command(t);

% Direct control signal
dela = (a_ref - Lp)/Ld*p(1) + b_ref/Ld*p_cmd + ke*( p(1) - p(2) );

% Compute RHS of plant
dp(1,1) = Lp*p(1) + Ld*dela;
dp(2,1) = a_ref*p(2) + b_ref*p_cmd;
