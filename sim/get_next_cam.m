function [feats, time_cam, succ] = get_next_cam(poses, featmap, true_bias, timestamp_last_cam)
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
sim = sim_config();

timestamp_last_imu = true_bias(end, 1);
% Return if the camera measurement should go before us
if (timestamp_last_cam + 1.0 / sim.freq_cam < timestamp_last_imu + 1.0 / sim.freq_imu)
    succ = false;
    return
end

timestamp_last_imu =  timestamp_last_imu + 1.0 / sim.freq_imu;
timestamp = timestamp_last_imu;
time_cam = timestamp_last_cam;

[R_GtoI, p_IinG, success_pose] = get_pose(poses, timestamp);

if (~success_pose)
    succ = false;
    return
end

uvs = projectPoints(featmap, p_IinG, R_GtoI, timestamp);

% If we do not have enough, generate more
if (length(uvs) < sim.num_pts)
    fprintf('[Simalator]: %d features generated at timestamp \n', sim.num_pts - length(uvs));
end

% If greater than only select the first set
if (length(uvs) > sim.num_pts)
    uvs = uvs(1:sim.num_pts, :);
end

% Loop through and add noise to each uv measurement
for i = 1:1:length(uvs)
    uvs(i).uv = uvs(i).uv + sim.sigma_pix * rand;
end

feats(end+1:end+length(uvs), 1) = uvs;

succ = true;
return

end
