function q = pitch_rate_command(t)

q = zeros(1, length(t));

% for i = 1 : length(t)
% 
%     % step commands in text
%     if t(i)>=1 && t(i)<=3
%         q(i) = 1;
%     elseif t(i)>=6 && t(i) <=8
%         q(i) = -1;
%     elseif t(i)>=12 && t(i)<=14
%         q(i) = 2;
%     elseif t(i)>= 17 && t(i)<=19
%         q(i) = -2;
%     elseif t(i)>=22 && t(i)<= 24
%         q(i) = 4;
%     elseif t(i)>=27 && t(i)<=29
%         q(i) = -4;
%     end
%     
% end

q = 2*sin(t');