# Robust_and_Adaptive_Control_for_Aerodynamic_Systems

Direct Model Reference Adaptive Control for MIMO Systems

Problem 9.2

Proved that if some of teh diagonal elements lambda_i of the unknown diagonal matrix Lambda_ in the system dynamics, then the adaptive laws

K_hat_dot_x = -Gamma_x * x * e' * P * B * sgn(Lambda)
K_hat_dot_r = -Gamma_r * r * e' * P * B * sgn(Lambda)
Theta_hat_dot = Gamma_Theta * Phi * e' * P * B * sgn(Lambda) 

solves the MIMO tracking problem. 

Implemented and simulated the system to test the MRAC controller in the presence of varios uncertianties and external commands of choice.
