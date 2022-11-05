% function VioManager()
% =========================================================================
% This function is use to manage the EKF system
% -------------------------------------------------------------------------
% Inputs :

% Outputs:

% -------------------------------------------------------------------------
% Copyright (C) 2022 @Yanyu Zhang, yzhan831@ucr.edu
% Copyright (C) 2022 @Jie Xu, jxu150@ucr.edu
% Copyright (C) 2022 @Wei Ren, ren@ece.ucr.edu
% =========================================================================
clear all;
close all;
clc;

% load trajectory
[poses, data] = load_simulated_trajectory('sim.txt');
% load config
cam = cam_config();
sim = sim_config();

% timestamp_start here is the 3rd timestamp in dataset
[data_gen, timestamp_start] = feed_trajectory(poses); 
% timestamp_last_cam = timestamp_start and true_bias = [3x7] t2 - t4
[featmap, uvs, true_bias, timestamp_last_cam] = simulator(poses);

% get the imu state at t4 (next_imu_time)
next_imu_time = timestamp_start + 1.0 / sim.freq_imu; % t4
[imustate, success] = get_state(data_gen, next_imu_time, true_bias);
if (~success)
    fprintf('[VioManager]: Could not initialize the filter to the first state. \n');
end

% Initialize our filter with the groundtruth at t4
[state_c, cov_c] = init_with_gt(imustate);

% get the imu measurement at t4
[R_GtoI, ~, w_IinI, ~, ~, a_IinG, success_accel] = get_acceleration(poses, next_imu_time);
a_IinI = R_GtoI * (a_IinG + [0;0;9.81]);
imu_cur.timestamp = next_imu_time;
imu_cur.wm = w_IinI;
imu_cur.am = a_IinI;

buffer_timecam = -1;
uvs_old = [];
state_est = [];
state_est(end+1, :) = state_c.';

plot3(data(:, 2), data(:, 3), data(:, 4));
hold on;

while (true)
    % t4 to t5
    [imu_next, true_bias_next, hasimu] = get_next_imu(poses, true_bias, timestamp_last_cam);

    % propagate from imu_cur to imu_next
    if (hasimu)
        [state_n, cov_n] = propagator(state_c, cov_c, imu_cur, imu_next);
    end

    state_est(end+1, :) = state_n.';

    plot3(state_n(5), state_n(6), state_n(7), '.');
    pause(0.1);

    state_c = state_n;
    cov_c = cov_n;
    imu_cur = imu_next;
    true_bias = true_bias_next;

%     [uvs_new, time_cam, hascam] = get_next_cam(poses, featmap, uvs_old, true_bias_new, timestamp_last_cam);
% 
%     update
% 
%     buffer_timecam = time_cam;
%     buffer_camids = camids;
%     buffer_feats = feats;

end







