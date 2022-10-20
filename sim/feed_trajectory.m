function [control_points, timestamp_start] = feed_trajectory(traj_points)
% =========================================================================
% This function is to feed in a series of poses that we will then convert 
% into control points.
% -------------------------------------------------------------------------
% Inputs :
% traj_points: the trajectory data
% Outputs:
% control_points: control points
% timestamp_start: timestamp start
% -------------------------------------------------------------------------
% Copyright (C) 2022 @Yanyu Zhang, yzhan831@ucr.edu
% Copyright (C) 2022 @Jie Xu, jxu150@ucr.edu
% Copyright (C) 2022 @Wei Ren, ren@ece.ucr.edu
% =========================================================================
% Find the average frequency to use as our uniform timesteps
sumdt = 0;
for i = 1:1:(length(traj_points)-1)
    sumdt = sumdt + traj_points(i + 1).timestamp - traj_points(i).timestamp;
end
dt = sumdt / (length(traj_points) - 1);

if dt < 0.05
    dt = 0.05;
end

timestamp_min = inf;
timestamp_max = -inf;

for i = 1:1:length(traj_points)
    if (traj_points(i).timestamp <= timestamp_min)
        timestamp_min = traj_points(i).timestamp;
    end
    if (traj_points(i).timestamp >= timestamp_max)
        timestamp_max = traj_points(i).timestamp;
    end
end

timestamp_curr = timestamp_min;
control_points = [];

while true
    [pose0, t0, pose1, t1, succ] = find_bounding_poses(traj_points, timestamp_curr);

    if (~succ)
        break;
    end

    lambda = (timestamp_curr - t0) / (t1 - t0);
    pose_interp = exp_se3(lambda * log_se3(pose1 * Inv_se3(pose0))) * pose0;

    new_pt.timestamp = timestamp_curr;
    new_pt.pose = pose_interp;

    control_points = [control_points; new_pt];
    timestamp_curr = timestamp_curr + dt;
end

timestamp_start = timestamp_min + 2 * dt;

end







