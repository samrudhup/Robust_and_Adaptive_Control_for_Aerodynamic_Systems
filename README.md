# Robust_and_Adaptive_Control_for_Aerodynamic_Systems

Aero1.m 
Analyses the longitudinal dynamics of a pransport aircraft, trimmed at V_0 = 250 ft/s, and flying at a low altitude. In the model, all angles and angular rates are in radians, airspeed is in ft/s, throttle is in lbs, and elevator deflections are in radians. Analysied the the open loop eigenvalues for both coupled and short period dynamics. 
Simulated the open-loop system response due to elevator and thrust step inputs. 

When the elevator is deflected trailing edge up to pitch the vehicle nose up, there is a small instant decrease in the vertical acceleration. Then, Az starts to increase, resulting in the aircraft pitch-up motion. This transient is caused by the elevator deflecting upward and creating a small negative lift increment. As a result, the vertical acceleration momentarily goes into the “wrong” direction before it reverses and builds up. These dynamics can also be explained by the fact that there is a nonminimum phase zero (with a positive real part) in the transfer function from de to Az . It is important to understand that all tail-driven aerial vehicles have similar characteristics. This phenomenon becomes very important during control design.


Aero1_p4.m

