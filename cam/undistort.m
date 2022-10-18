function pt_out = undistort(uv_dist, cam)
% =========================================================================
% This function is meant to undistort uv point based on the cameara model
% -------------------------------------------------------------------------
% Inputs :
% uv_dist: distort uv
%   cam  : camera parameters
% Outputs:
% pt_out : undistorted point
% -------------------------------------------------------------------------
% Copyright (C) 2022 @Yanyu Zhang, yzhan831@ucr.edu
% Copyright (C) 2022 @Jie Xu, jxu150@ucr.edu
% Copyright (C) 2022 @Wei Ren, ren@ece.ucr.edu
% =========================================================================
camK = [cam.intrinsics(1), 0, cam.intrinsics(3);
        0, cam.intrinsics(2), cam.intrinsics(4);
        0, 0, 1];
camD = cam.distortion_coeffs;

params = cameraParameters("K", camK, "RadialDistortion", camD(1:2), ...
                          "TangentialDistortion", camD(3:4));
pt_out = undistortPoints(uv_dist, params);

end

