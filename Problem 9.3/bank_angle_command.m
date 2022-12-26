function phi_cmd_deg = bank_angle_command(t)

phi_cmd_deg = zeros(size(t));

for i = 1 : length(t)

    % step commands in text
    if t(i)>=1 && t(i)<=10
        phi_cmd_deg(i) = 5;
    elseif t(i)>=22 && t(i) <=27
        phi_cmd_deg(i) = -5;
    elseif t(i)>=43 && t(i)<=49
        phi_cmd_deg(i) = 10;
    elseif t(i)>= 63 && t(i)<=73
        phi_cmd_deg(i) = -10;
    elseif t(i)>=85 && t(i)<= 95
        phi_cmd_deg(i) = 5;
    elseif t(i)>=105 && t(i)<=115
        phi_cmd_deg(i) = -5;
    end
end