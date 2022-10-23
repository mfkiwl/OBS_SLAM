function [R_GtoI, p_IinG, found] = get_pose(poses, timestamp)
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
    R_GtoI = eye(3);
    p_IinG = zeros(3, 1);
    found = false;
    return
end

DT = (t2 - t1);
u = (timestamp - t1) / DT;
b0 = 1.0 / 6.0 * (5 + 3 * u - 3 * u * u + u * u * u);
b1 = 1.0 / 6.0 * (1 + 3 * u + 3 * u * u - 2 * u * u * u);
b2 = 1.0 / 6.0 * (u * u * u);

A0 = exp_se3(b0 * log_se3(Inv_se3(pose0) * pose1));
A1 = exp_se3(b1 * log_se3(Inv_se3(pose1) * pose2));
A2 = exp_se3(b2 * log_se3(Inv_se3(pose2) * pose3));

pose_interp = pose0 * A0 * A1 * A2;
R_GtoI = pose_interp(1:3, 1:3).';
p_IinG = pose_interp(1:3, 4);

found = true;
return

end
