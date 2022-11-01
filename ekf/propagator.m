function [state_n, cov_n] = propagator(state_c, cov_c, imu_begin, imu_end)
% =========================================================================
% This function is meant to propagate IMU from begin to end
% -------------------------------------------------------------------------
% Inputs :
% state_c: current state vector
%  cov_c : current covariance matrix
%imu_begin: imu measurement with structure imu.am, imu.wm, and
% imu.timestamp
% imu_end: imu measurement with structure imu.am, imu.wm, and 
% imu.timestamp
% Outputs:
% state_n: next state vector
%  cov_n : next covariance matrix
%   succ : T/F
% -------------------------------------------------------------------------
% Copyright (C) 2022 @Yanyu Zhang, yzhan831@ucr.edu
% Copyright (C) 2022 @Jie Xu, jxu150@ucr.edu
% Copyright (C) 2022 @Wei Ren, ren@ece.ucr.edu
% =========================================================================
sim = sim_config();
sigma = [sim.sigma_g, sim.sigma_a, sim.sigma_gb, sim.sigma_ab].';

dt = imu_end.timestamp - imu_begin.timestamp;

R_GtoI = quat2rot(state_c(1:4));
% p_IinG = state_c(5:7);
% v_IinG = state_c(8:10);
bg = state_c(11:13);
ba = state_c(14:16);
feats = [];
if length(state_c) > 16
    feats = state_c(17:end);
end
% g = [0;0;9.81];

% Corrected imu measurements
w_hat = imu_begin.wm - bg;
a_hat = imu_begin.am - ba;
w_hat2 = imu_end.wm - bg;
a_hat2 = imu_end.am - ba;

% State transition and noise matrix
Phi11 = exp_so3(-w_hat * dt);
Phi21 = -0.5 * R_GtoI.' * skew_x(a_hat * dt * dt);
Phi31 = -R_GtoI.' * skew_x(a_hat * dt);
Phi22 = eye(3);
Phi23 = eye(3) * dt;
Phi33 = eye(3);
Phi14 = -exp_so3(-w_hat * dt) * Jr_so3(-w_hat * dt) * dt;
Phi44 = eye(3);
Phi25 = -0.5 * R_GtoI.' * dt * dt;
Phi35 = -R_GtoI.' * dt;
Phi55 = eye(3);

Phi = [Phi11, zeros(3), zeros(3), Phi14, zeros(3);
       Phi21, Phi22, Phi23, zeros(3), Phi25;
       Phi31, zeros(3), Phi33, zeros(3), Phi35;
       zeros(3), zeros(3), zeros(3), Phi44, zeros(3);
       zeros(3), zeros(3), zeros(3), zeros(3), Phi55];

G11 = -exp_so3(-w_hat * dt) * Jr_so3(-w_hat * dt) * dt;
G22 = -0.5 * R_GtoI.' * dt * dt;
G32 = -R_GtoI.' * dt;
G43 = eye(3);
G54 = eye(3);

G = [G11, zeros(3), zeros(3), zeros(3);
     zeros(3), G22, zeros(3), zeros(3);
     zeros(3), G32, zeros(3), zeros(3);
     zeros(3), zeros(3), G43, zeros(3);
     zeros(3), zeros(3), zeros(3), G54];

if length(feats) >= 1
    l = length(feats);
    Phi = blkdiag(Phi, zeros(l));
    G = [G; zeros(l, 12)];
end

% Construct our discrete noise covariance matrix
Qc = zeros(12, 12);
Qc(1:3, 1:3) = sigma(1)^2 / dt * eye(3);
Qc(4:6, 4:6) = sigma(2)^2 / dt * eye(3);
Qc(7:9, 7:9) = sigma(3)^2 * dt * eye(3);
Qc(10:12, 10:12) = sigma(4)^2 * dt * eye(3);
Qd = G * Qc * G.';
Qd = 0.5 * (Qd + Qd.');
cov_n = Phi * cov_c * Phi.' + Qd;

% % quaternion propagate
% state_n(1:4) = rot2quat(exp_so3(-w_hat * dt) * R_Gtoi);
% % position propagate
% state_n(5:7) = p_IinG + v_IinG * dt + 0.5 * R_Gtoi.' * a_hat * dt * dt - 0.5 * g * dt * dt;
% % velocity propagate
% state_n(8:10) = v_IinG + R_GtoI.' * a_hat * dt - g * dt;

% Propagate to next state
[new_q, new_v, new_p] = predict_mean_discrete(state_c, dt, w_hat, a_hat, w_hat2, a_hat2);
state_n(1:4) = new_q;
state_n(5:7) = new_p;
state_n(8:10) = new_v;

end
