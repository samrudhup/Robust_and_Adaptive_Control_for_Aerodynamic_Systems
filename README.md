# Robust_and_Adaptive_Control_for_Aerodynamic_Systems

Problem 7.1

Given the roll damping L_p = -0.8(s-1) and the aileron effectiveness L_delta_a = 1.6 (s-1), 
design a fixed-gain model reference controller in the form of delta_a = ((a_ref - L_p)/L_dela)*p + (b_ref/L_dela)*p_cmd to recover the reference model dynamics p_ref = a_ref * p + b_ref * p_cmd,
with a_ref =-2,b_ref = 2.

Also,designed a fixed-gain controller with error feedback in the form of delta_a = kp*p + k_pcmd * p_cmd - ke * (p - p_ref). 

Choose several bounded time-varying roll rate commands. Simulated the closed-loop system response, with each of the two controllers active (one at a time). 

Compared the two controllers and the achieved closed-loop system stability, robustness, tracking, and transient properties.


