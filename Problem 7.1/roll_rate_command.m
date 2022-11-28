function y = roll_rate_command(t)

y = zeros(1, length(t));

% step command
% y(t>2) = 1;

% ramp command
% y(t<=2&t>=1) = t(t<=2&t>=1)-1; 
% y(t>2) = 1;

% sinusoid command
% y = sin(0.5*pi*t);
y = sin(2*pi*t).*cos(.3*pi*t+.2) + 0.5*cos(.14*pi*t);