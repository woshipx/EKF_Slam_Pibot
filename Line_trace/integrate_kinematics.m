function state_t = integrate_kinematics(state_t, dt, u, q)
%INTEGRATE_KINEMATICS integrate the kinematics of a robot in 2D
%   state is the current state, and has the form [x;y;theta]
%   dt is the length of time to integrate
%   u is the forward speed input
%   q is the angular speed input
%   new_state is the state after integration, also in the form [x;y;theta]

% integration kinematics
if q ~= 0
    state_t(1) = state_t(1) + u*( sin(state_t(3)+q*dt) - sin(state_t(3)))/q;
    state_t(2) = state_t(2) + u*( -cos(state_t(3)+q*dt) + cos(state_t(3)))/q;
    state_t(3) = state_t(3) + q*dt;
else
    state_t(1) = state_t(1) + u*cos(state_t(3))*dt;
    state_t(2) = state_t(2) + u*sin(state_t(3))*dt;
    state_t(3) = state_t(3) + 0;
end

end
