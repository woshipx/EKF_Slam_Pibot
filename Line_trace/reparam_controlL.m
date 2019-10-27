function [wheel_velocities] = reparam_controlL(u, q)
%REPARAM_CONTROL convert control commands for the pibot

%   u is the desired forward speed
%   q is the desired angular speed

%   wheel_velocities is a vector of the required wheel velocities the form
%       [left_wheel_vel; right_wheel_vel].


baseline = 0.149; % m
scale = 0.0051;
L = round((-q*baseline/2 + u)/(scale));
R = round((q*baseline/2 + u)/(scale));

% L = (-q*baseline./2 + u)./(scale);
% R = (q*baseline./2 + u)./(scale);

wheel_velocities = [L;R];

end
