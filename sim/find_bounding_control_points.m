function [pose0, t0, pose1, t1, pose2, t2, pose3, t3, found] = find_bounding_control_points(poses, timestamp)
% =========================================================================
% This function is to find the upper bounding and the lower bounding of 
% poses of the current timestamp
% -------------------------------------------------------------------------
% Inputs :
%  poses : the trajectory data
% timestamp: current timestamp
% Outputs:
%  pose0 : SE(3) pose of the first pose
%    t0  : timestamp of the first pose
%  pose1 : SE(3) pose of the second pose
%    t1  : timestamp of the second pose
%  pose2 : SE(3) pose of the third pose
%    t2  : timestamp of the third pose
%  pose3 : SE(3) pose of the fourth pose
%    t3  : timestamp of the fourth pose
%  found : T/F
% -------------------------------------------------------------------------
% Copyright (C) 2022 @Yanyu Zhang, yzhan831@ucr.edu
% Copyright (C) 2022 @Jie Xu, jxu150@ucr.edu
% Copyright (C) 2022 @Wei Ren, ren@ece.ucr.edu
% =========================================================================
t0 = -1;
t3 = -1;
pose0 = eye(4);
pose3 = eye(4);
found = false;

[pose1, t1, pose2, t2, succ] = find_bounding_poses(poses, timestamp);

if succ && (t1 ~= poses(1).timestamp) && (t2 ~= poses(end).timestamp)
    all_t = [poses.timestamp];
    tf1 = all_t == t1;
    tf2 = all_t == t2;
    index1 = find(tf1);
    index2 = find(tf2);
    t0 = poses(index1 - 1).timestamp;
    pose0 = poses(index1 - 1).pose;
    t3 = poses(index2 + 1).timestamp;
    pose3 = poses(index2 + 1).pose;
end

if (t0 < t1) && (t1 < t2) && (t2 < t3)
    found = true;
end

end

