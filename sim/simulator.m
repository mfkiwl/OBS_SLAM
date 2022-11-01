function [featmap_c, uv_all, true_bias, timestamp_last_cam] = simulator(data)
% =========================================================================
% This function is to run the simulation
% -------------------------------------------------------------------------
% Inputs :
%  data  : the trajectory data
% Outputs:
% featmap: a list of features in 3D
% -------------------------------------------------------------------------
% Copyright (C) 2022 @Yanyu Zhang, yzhan831@ucr.edu
% Copyright (C) 2022 @Jie Xu, jxu150@ucr.edu
% Copyright (C) 2022 @Wei Ren, ren@ece.ucr.edu
% =========================================================================
sim = sim_config();
[data_gen, timestamp_start] = feed_trajectory(data);

timestamp = timestamp_start;
timestamp_last_imu = timestamp_start;
timestamp_last_cam = timestamp_start;

[R_GtoI_init, p_IinG_init, success_pose_init] = get_pose(data_gen, timestamp);

if (~success_pose_init)
    fprintf('[Simalator]: Unable to find the first pose in the spline. \n');
end

% Find the timestamp that we move enough to be considered "moved"
distance = 0.0;
distancethreshold = sim.distance_threshold;
while (true)

    % Get the pose at the current timestep
    [R_GtoI, p_IinG, success_pose] = get_pose(data_gen, timestamp);

    if (~success_pose)
        fprintf('[Simalator]: unable to find jolt in the groundtruth data to initialize at. \n');
    end

    distance = distance + norm(p_IinG - p_IinG_init);
    p_IinG_init = p_IinG;

    if (distance > distancethreshold)
        break;
    else
        timestamp = timestamp + 1.0 / sim.freq_cam;
        timestamp_last_imu = timestamp_last_imu + 1.0 / sim.freq_cam;
        timestamp_last_cam = timestamp_last_cam + 1.0 / sim.freq_cam;
    end
end

hist_true_bias_time = [];
hist_true_bias_accel = [];
hist_true_bias_gyro = [];

% Append the current true bias to our history
hist_true_bias_time(end + 1, 1) = timestamp_last_imu - 1.0 / sim.freq_imu;
hist_true_bias_accel(end + 1, 1:3) = sim.true_bias_accel.';
hist_true_bias_gyro(end + 1, 1:3) = sim.true_bias_gyro.';

hist_true_bias_time(end + 1, 1) = timestamp_last_imu;
hist_true_bias_accel(end + 1, 1:3) = sim.true_bias_accel.';
hist_true_bias_gyro(end + 1, 1:3) = sim.true_bias_gyro.';

hist_true_bias_time(end + 1, 1) = timestamp_last_imu + 1.0 / sim.freq_imu;
hist_true_bias_accel(end + 1, 1:3) = sim.true_bias_accel.';
hist_true_bias_gyro(end + 1, 1:3) = sim.true_bias_gyro.';

true_bias = [hist_true_bias_time, hist_true_bias_gyro, hist_true_bias_accel];

uv_all = [];
featmap_n = [];
time_synth = timestamp_start;

% Loop through each pose and generate our feature map in them!!!!
while (true)
    [R_GtoI, p_IinG, succ] = get_pose(data_gen, time_synth);
    featmap_c = featmap_n;

    if (~succ)
        break;
    end
    
    uvs = projectPoints(featmap_c, p_IinG, R_GtoI, time_synth);
    uv_all = [uv_all; uvs];
    
    if (length(uvs) < sim.num_pts)
        featmap_n = generatePoints(featmap_c, p_IinG, R_GtoI, sim.num_pts - length(uvs));
        fprintf('[Simalator]: %d features generated at timestamp %f\n', sim.num_pts - length(uvs), time_synth);
    end

    time_synth = time_synth + sim.dt;

end

fprintf('[Simalator]: The total number of feature is %d \n', length(featmap_n));
fprintf('[Simalator]: The total number of timestamp is %d \n', (time_synth-timestamp_start)/sim.dt);
pause(0.1);

end
