function [new_state,new_P] = propagation(state,P,dt,imu,sigma_a,sigma_g,sigma_wa,sigma_wg)
% =========================================================================
% This function is meant to imu propagation in EKF
% -------------------------------------------------------------------------
% Inputs :
%    w   : a 3x1 vector
% Outputs:
%    R   : a 3x3 matrix
% -------------------------------------------------------------------------
% Copyright (C) 2022 @Yanyu Zhang, yzhan831@ucr.edu
% Copyright (C) 2022 @Jie Xu, jxu150@ucr.edu
% Copyright (C) 2022 @Wei Ren, ren@ece.ucr.edu
% =========================================================================

% imu is a struct with imu(k).a_I_m, imu(k).omega_I_m, k is the current
% time step
% initialize
% sigma_a = eye(3)*0.1;
% sigma_g = eye(3)*0.1;
% sigma_wa = eye(3)*0.01;
% sigma_wg = eye(3)*0.01;

new_state = state;
new_P = P;
% separate states
quat = state(1:4);
p = state(5:7);
v = state(8:10);
bg = state(11:13);
ba = state(14:16);
features = [];
len = length(state);
if len>16
    features = state(17:end);
end
% get imu measurements
w_hat = imu.omega_I_m - bg;
a_hat = imu.a_I_m;
g = [0;0;9.81];
R_G_I = quattorot(quat); %from G to I
% quaternion propagate
new_quat = expm(0.5*Omega(w_hat)*dt) * quat;
new_state(1:4) = new_quat/norm(new_quat);
%position propagate
new_state(5:7) = p + v * dt + 0.5 * R_G_I' * a_hat * dt^2-0.5 * g * dt^2;
% velocity propagate
new_state(8:10) = v +  R_G_I' * a_hat * dt - g * dt;

% covariance propagate
phi_qq = expm(matskew(-w_hat*dt));
phi_qbg = -R_G_I'*Jr_so3(-w_hat*dt)*dt;

phi_pq = -0.5 * R_G_I'  * matskew(a_hat * dt^2);
phi_pp = eye(3);
phi_pv = dt*eye(3);
phi_pba = -0.5*R_G_I'*dt^2;

phi_vq = -R_G_I' * matskew(a_hat * dt);
phi_vv = eye(3);
phi_vba = -R_G_I'*dt;

phi_bgbg = eye(3);
phi_baba = eye(3);

Phi = [phi_qq   zeros(3) zeros(3) phi_qbg  zeros(3);
       phi_pq   phi_pp   phi_pv   zeros(3) phi_pba;
       phi_vq   zeros(3) phi_vv   zeros(3) phi_vba;
       zeros(3) zeros(3) zeros(3) phi_bgbg zeros(3);
       zeros(3) zeros(3) zeros(3) zeros(3) phi_baba];

G = [phi_qbg   zeros(3) zeros(3)  zeros(3);
     zeros(3)  phi_pba  zeros(3)  zeros(3);
     zeros(3)  phi_vba  zeros(3)  zeros(3);
     zeros(3)  zeros(3) eye(3) zeros(3);
     zeros(3)  zeros(3) zeros(3)  eye(3)];
if length(features)>1
    l = length(features);
    Phi = blkdiag(Phi,eye(l));
    G = [G;zeros(l,12)];
end

Q_d = G*[(sigma_g^2/dt).*eye(3) zeros(3) zeros(3) zeros(3);
            zeros(3) (sigma_a^2/dt).*eye(3) zeros(3) zeros(3);
            zeros(3) zeros(3) (sigma_wg^2/dt).*eye(3) zeros(3);
            zeros(3) zeros(3)  zeros(3) (sigma_wa^2/dt).*eye(3)]*G';

Q_d = 0.5*(Q_d+Q_d');

new_P = Phi * P * Phi' +  Q_d ;
new_P = (new_P + new_P')/2;
end

