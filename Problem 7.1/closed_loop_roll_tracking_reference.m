function dp = closed_loop_roll_tracking_reference(t, p, a_ref, b_ref)

p_cmd = roll_rate_command(t);

% Compute RHS of plant
dp = a_ref*p + b_ref*p_cmd;