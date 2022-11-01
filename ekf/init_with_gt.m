function [state, Cov] = init_with_gt(imustate)
% =========================================================================
% This function is used to initialize the imu state from groundtruth
% -------------------------------------------------------------------------
% Inputs :
% imustate: init imu state
% Outputs:
%  state : init state
%   Cov  : init Cov
% -------------------------------------------------------------------------
% Copyright (C) 2022 @Yanyu Zhang, yzhan831@ucr.edu
% Copyright (C) 2022 @Jie Xu, jxu150@ucr.edu
% Copyright (C) 2022 @Wei Ren, ren@ece.ucr.edu
% =========================================================================
state.imu = imustate; % 16x1

Cov = 0.02^2 * eye(16);
Cov(1:3, 1:3) = 0.017^2 * eye(3); % q
Cov(4:6, 4:6) = 0.05^2 * eye(3); % p
Cov(7:9, 7:9) = 0.01^2 * eye(3); % v (static)

state.timestamp = imustate.timestamp;

end