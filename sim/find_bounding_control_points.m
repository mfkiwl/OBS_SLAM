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
t1 = -1;
t2 = -1;
t3 = -1;
pose0 = eye(4);
pose1 = eye(4);
pose2 = eye(4);
pose3 = eye(4);

[pose1, t1, pose2, t2, succ] = find_bounding_poses(poses, timestamp);

if ~succ
    found = false;
    return
end

for i = 1:1:length(poses)
    if (t1 == poses(i).timestamp)
        iter_t1 = i;
        break;
    end
end

for j = 1:1:length(poses)
    if (t2 == poses(j).timestamp)
        iter_t2 = j;
        break;
    end
end

if (iter_t1 == 1) 
    found = false;
    return
end

iter_t0 = iter_t1 - 1;
iter_t3 = iter_t2 + 1;

if (iter_t3 == length(poses)) 
    found = false;
    return
end

t0 = poses(iter_t0).timestamp;
pose0 = poses(iter_t0).pose;

t3 = poses(iter_t3).timestamp;
pose3 = poses(iter_t3).pose;

if (succ) 
    assert(t0 < t1);
    assert(t1 < t2);
    assert(t2 < t3);
end

found = true;
return

end

