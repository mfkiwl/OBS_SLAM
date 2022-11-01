function VioManager()
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
% load trajectory
[data_T, data] = load_simulated_trajectory('udel_gore.txt');
% load config
cam = cam_config();
sim = sim_config();

[data_gen, timestamp_start] = feed_trajectory(data_T);
[featmap, uvs, true_bias, timestamp_last_cam] = simulator(data_T);

next_imu_time = timestamp_start + 1.0 / sim.freq_imu;
[imustate, success] = get_state(data_gen, next_imu_time, true_bias);

if (~success)
    fprintf('[VioManager]: Could not initialize the filter to the first state. \n');
end

% Initialize our filter with the groundtruth
[state_init, cov_init] = init_with_gt(imustate);

buffer_timecam = -1;

while (true)
    [time_imu, wm, am, true_bias_new, hasimu] = get_next_imu(poses, true_bias, timestamp_last_cam);

    [state_n, cov_n] = propagator(state_c, cov_c, imu_begin, imu_end)


end



end



