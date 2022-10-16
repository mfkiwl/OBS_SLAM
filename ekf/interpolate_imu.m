function imudata = interpolate_imu(imu_1, imu_2, time)
% =========================================================================
% This function is meant to linearly interpolate between two imu messages
% -------------------------------------------------------------------------
% Inputs :
% imu_1  : imu measurement 1
% imu_2  : imu measurement 2
%  time  : time
% Outputs:
% imudata: data
% -------------------------------------------------------------------------
% Copyright (C) 2022 @Yanyu Zhang, yzhan831@ucr.edu
% Copyright (C) 2022 @Jie Xu, jxu150@ucr.edu
% Copyright (C) 2022 @Wei Ren, ren@ece.ucr.edu
% =========================================================================
lambda = (time - imu_1.timestamp) / (imu_2.timestamp - imu_1.timestamp);
imudata.timestamp = timestamp;
imudata.am = (1 - lambda) * imu_1.am + lambda * imu_2.am;
imudata.wm = (1 - lambda) * imu_1.wm + lambda * imu_2.wm;
end