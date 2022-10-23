function sim = sim_config()
% =========================================================================
% This function is meant to set the simulation config
% -------------------------------------------------------------------------
% Outputs:
%   sim  : simulation config
% -------------------------------------------------------------------------
% Copyright (C) 2022 @Yanyu Zhang, yzhan831@ucr.edu
% Copyright (C) 2022 @Jie Xu, jxu150@ucr.edu
% Copyright (C) 2022 @Wei Ren, ren@ece.ucr.edu
% =========================================================================
sim.num_pts = 10;

sim.distance_threshold = 1.2;

% Frequency (Hz) that we will simulate our cameras
sim.freq_cam = 10.0;

% Frequency (Hz) that we will simulate our inertial measurement unit
sim.freq_imu = 400.0;

% Feature distance we generate features from (minimum)
sim.min_depth = 5;
sim.min_feature_gen_distance = 5;

% Feature distance we generate features from (maximum)
sim.max_depth = 10;
sim.max_feature_gen_distance = 10;

% Gyroscope white noise (rad/s/sqrt(hz))
sim.sigma_g = 1.6968e-04;

% Gyroscope random walk (rad/s^2/sqrt(hz))
sim.sigma_gb = 1.9393e-05;

% Accelerometer white noise (m/s^2/sqrt(hz))
sim.sigma_a = 2.0000e-3;

% Accelerometer random walk (m/s^3/sqrt(hz))
sim.sigma_ab = 3.0000e-03;

end