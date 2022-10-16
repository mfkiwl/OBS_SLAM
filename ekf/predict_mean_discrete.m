function [new_q, new_v, new_p] = predict_mean_discrete(state, dt, w_hat1, a_hat1, w_hat2, a_hat2)
% IMU average
w_hat = .5 * (w_hat1 + w_hat2);
a_hat = .5 * (a_hat1 + a_hat2);

% Pre-compute things
w_norm = w_hat.norm();
R_GtoI = quat2rot(state(1:4));
p_IinG = state(5:7);
v_IinG = state(8:10);

g = [0;0;9.81];

if (w_norm > 1e-20)
    bigO = cos(0.5 * w_norm * dt) * eye(4) + 1 / w_norm * sin(0.5 * w_norm * dt) * Omega(w_hat);
else
    bigO = eye(4) + 0.5 * dt * Omega(w_hat);
end

new_q = quatnorm(bigO * state(1:4));
% new_q = rot2quat(exp_so3(-w_hat * dt) * R_Gtoi);

new_v = v_IinG + R_GtoI.' * a_hat * dt - g * dt;

new_p = p_IinG + v_IinG * dt + 0.5 * R_GtoI.' * a_hat * dt * dt - 0.5 * g * dt * dt;

end