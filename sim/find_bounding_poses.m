function [pose0, t0, pose1, t1, found] = find_bounding_poses(data, timestamp)
% =========================================================================
% This function is to find the upper bounding and the lower bounding of 
% poses of the current timestamp
% -------------------------------------------------------------------------
% Inputs :
%  data  : the trajectory data
% timestamp: current timestamp
% Outputs:
%  pose0 : lower bounding pose
%    t0  : lower bounding time
%  pose1 : upper bounding pose
%    t1  : upper bounding time
%  found : T/F
% -------------------------------------------------------------------------
% Copyright (C) 2022 @Yanyu Zhang, yzhan831@ucr.edu
% Copyright (C) 2022 @Jie Xu, jxu150@ucr.edu
% Copyright (C) 2022 @Wei Ren, ren@ece.ucr.edu
% =========================================================================
for i = 1:1:(length(data)-1)
    if (data(i).timestamp == timestamp)
        if i ~= 1
            t0 = data(i-1).timestamp;
            t1 = data(i+1).timestamp;
            pose0 = data(i-1).pose;
            pose1 = data(i+1).pose;
            found = true;
            break;
        else
            t0 = data(i).timestamp;
            t1 = data(i+1).timestamp;
            pose0 = data(i).pose;
            pose1 = data(i+1).pose;
            found = true;
            break;
        end

    elseif (data(i).timestamp < timestamp) && (data(i+1).timestamp > timestamp)
        t0 = data(i).timestamp;
        t1 = data(i+1).timestamp;
        pose0 = data(i).pose;
        pose1 = data(i+1).pose;
        assert((t0 < t1),'find_bounding_poses: t0 < t1?');
        found = true;
        break

    else
        t0 = -1;
        t1 = -1;
        pose0 = eye(4);
        pose1 = eye(4);
        found = false;
    end
end

end