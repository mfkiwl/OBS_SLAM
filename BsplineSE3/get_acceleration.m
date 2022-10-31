function [R_GtoI, p_IinG, w_IinI, v_IinG, alpha_IinI, a_IinG, found] = get_acceleration(poses, timestamp)
% =========================================================================
% This function is to find the upper bounding and the lower bounding of 
% poses of the current timestamp
% -------------------------------------------------------------------------
% Inputs :
%  poses : the trajectory data
% timestamp: current timestamp
% Outputs:
%  R_GtoI: SO(3) pose 
%  p_IinG: relative position
%  found : T/F
% -------------------------------------------------------------------------
% Copyright (C) 2022 @Yanyu Zhang, yzhan831@ucr.edu
% Copyright (C) 2022 @Jie Xu, jxu150@ucr.edu
% Copyright (C) 2022 @Wei Ren, ren@ece.ucr.edu
% =========================================================================
[pose0, t0, pose1, t1, pose2, t2, pose3, t3, succ] = find_bounding_control_points(poses, timestamp);

if (~succ)
    alpha_IinI = zeros(3, 1);
    a_IinG = zeros(3, 1);
    found = false;
    return
end

DT = (t2 - t1);
u = (timestamp - t1) / DT;
b0 = 1.0 / 6.0 * (5 + 3 * u - 3 * u * u + u * u * u);
b1 = 1.0 / 6.0 * (1 + 3 * u + 3 * u * u - 2 * u * u * u);
b2 = 1.0 / 6.0 * (u * u * u);
b0dot = 1.0 / (6.0 * DT) * (3 - 6 * u + 3 * u * u);
b1dot = 1.0 / (6.0 * DT) * (3 + 6 * u - 6 * u * u);
b2dot = 1.0 / (6.0 * DT) * (3 * u * u);
b0dotdot = 1.0 / (6.0 * DT * DT) * (-6 + 6 * u);
b1dotdot = 1.0 / (6.0 * DT * DT) * (6 - 12 * u);
b2dotdot = 1.0 / (6.0 * DT * DT) * (6 * u);

omega_10 = log_se3(Inv_se3(pose0) * pose1);
omega_21 = log_se3(Inv_se3(pose1) * pose2);
omega_32 = log_se3(Inv_se3(pose2) * pose3);
omega_10_hat = hat_se3(omega_10);
omega_21_hat = hat_se3(omega_21);
omega_32_hat = hat_se3(omega_32);

A0 = exp_se3(b0 * omega_10);
A1 = exp_se3(b1 * omega_21);
A2 = exp_se3(b2 * omega_32);
A0dot = b0dot * hat_se3(omega_10) * A0;
A1dot = b1dot * hat_se3(omega_21) * A1;
A2dot = b2dot * hat_se3(omega_32) * A2;
A0dotdot = b0dot * omega_10_hat * A0dot + b0dotdot * omega_10_hat * A0;
A1dotdot = b1dot * omega_21_hat * A1dot + b1dotdot * omega_21_hat * A1;
A2dotdot = b2dot * omega_32_hat * A2dot + b2dotdot * omega_32_hat * A2;

pose_interp = pose0 * A0 * A1 * A2;
R_GtoI = pose_interp(1:3, 1:3).';
p_IinG = pose_interp(1:3, 4);

vel_interp = pose0 * (A0dot * A1 * A2 + A0 * A1dot * A2 + A0 * A1 * A2dot);
w_IinI = vee(pose_interp(1:3, 1:3).' * vel_interp(1:3, 1:3));
v_IinG = vel_interp(1:3, 4);

acc_interp = pose0 * (A0dotdot * A1 * A2 + A0 * A1dotdot * A2 + ...
             A0 * A1 * A2dotdot + 2 * A0dot * A1dot * A2 + ...
             2 * A0 * A1dot * A2dot + 2 * A0dot * A1 * A2dot);
omegaskew = pose_interp(1:3, 1:3).' * vel_interp(1:3, 1:3);
alpha_IinI = vee(pose_interp(1:3, 1:3).' * (acc_interp(1:3, 1:3) - vel_interp(1:3, 1:3) * omegaskew));
a_IinG = acc_interp.block(1:3, 4);

found = true;
return

end