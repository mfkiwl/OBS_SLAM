function [p_FinG, success] = triangulation(meas1, meas2)
% =========================================================================
% This function is meant to triangulation features based on the first two
% measurements
% -------------------------------------------------------------------------
% Inputs :

% Outputs:

% -------------------------------------------------------------------------
% Copyright (C) 2022 @Yanyu Zhang, yzhan831@ucr.edu
% Copyright (C) 2022 @Jie Xu, jxu150@ucr.edu
% Copyright (C) 2022 @Wei Ren, ren@ece.ucr.edu
% =========================================================================
if norm(meas1.uv - meas2.uv) < 10 % uv displacement check
    success = false;
    return;
end

A = zeros(3);
b = zeros(3, 1);



cam = cam_config();

