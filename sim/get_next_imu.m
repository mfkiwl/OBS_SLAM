function [time_imu, wm, am, true_bias_new, succ] = get_next_imu(poses, true_bias, timestamp_last_cam)
% =========================================================================
% This function is to get the next imu
% -------------------------------------------------------------------------
% Inputs :
%  poses : the trajectory data
% desired_time: current timestamp
% Outputs:

%  found : T/F
% -------------------------------------------------------------------------
% Copyright (C) 2022 @Yanyu Zhang, yzhan831@ucr.edu
% Copyright (C) 2022 @Jie Xu, jxu150@ucr.edu
% Copyright (C) 2022 @Wei Ren, ren@ece.ucr.edu
% =========================================================================
timestamp_last_imu = true_bias(end, 1);
% Return if the camera measurement should go before us
if (timestamp_last_cam + 1.0 / sim.freq_cam < timestamp_last_imu + 1.0 / sim.freq_imu)
    succ = false;
    return
end

timestamp_last_imu =  timestamp_last_imu + 1.0 / sim.freq_imu;
timestamp = timestamp_last_imu;
time_imu = timestamp_last_imu;

[R_GtoI, p_IinG, w_IinI, v_IinG, alpha_IinI, a_IinG, success_accel] = get_acceleration(poses, timestamp);

if (~success_accel)
    succ = false;
    return
end

omega_inI = w_IinI;
gravity = [0;0;9.81];

accel_inI = R_GtoI * (a_IinG + gravity);

dt = 1.0 / sim.freq_imu;

true_bias_gyro = sim.true_bias_gyro + sim.sigma_gb * sqrt(dt) * rand;
true_bias_accel = sim.true_bias_accel + sim.sigma_ab * sqrt(dt) * rand;

% Append the current true bias to our history
true_bias_new = true_bias;
true_bias_new(end+1, 1) = timestamp_last_imu;
true_bias_new(end, 2:4) = true_bias_gyro.';
true_bias_new(end, 5:7) = true_bias_accel.';

wm = omega_inI + sim.true_bias_gyro + sim.sigma_g / sqrt(dt) * rand;
am = accel_inI + sim.true_bias_accel + sim.sigma_a / sqrt(dt) * rand;

succ = true;

end