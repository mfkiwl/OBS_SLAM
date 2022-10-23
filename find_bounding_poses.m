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
t0 = -1;
t1 = -1;
pose0 = eye(4);
pose1 = eye(4);

found_older = false;
found_newer = false;

for i = 1:1:length(data)
    if (data(i).timestamp >= timestamp) 
%         && (data(i-1).timestamp < timestamp)
          lower_bound = i;
          break;
    end
end

for j = 1:1:length(data)
    if (data(j).timestamp > timestamp) 
%         && (data(j-1).timestamp <= timestamp)
          upper_bound = j;
          break;
    end
end

if (lower_bound ~= length(data)) 
    if (data(lower_bound).timestamp == timestamp)
        found_older = true;
    elseif (lower_bound ~= 2)
        lower_bound = lower_bound - 1;
        found_older = true;
    end
end

if (upper_bound ~= length(data))
    found_newer = true;
end

if (found_older)
    t0 = data(lower_bound).timestamp;
    pose0 = data(lower_bound).pose;
end


if (found_newer)
    t1 = data(upper_bound).timestamp;
    pose1 = data(upper_bound).pose;
end

if (found_older && found_newer)
    assert(t0 < t1, "t0 is not smaller than t1");
end

found = (found_older && found_newer);

end