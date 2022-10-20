function pt_out = undistort(uv_dist)
% =========================================================================
% This function is meant to undistort uv point based on the cameara model
% -------------------------------------------------------------------------
% Inputs :
% uv_dist: distort uv 1 x 2 matrix 
%   cam  : camera parameters
% Outputs:
% pt_out : undistorted point 1 x 2 matrix 
% -------------------------------------------------------------------------
% Copyright (C) 2022 @Yanyu Zhang, yzhan831@ucr.edu
% Copyright (C) 2022 @Jie Xu, jxu150@ucr.edu
% Copyright (C) 2022 @Wei Ren, ren@ece.ucr.edu
% =========================================================================
cam = cam_config();

fx = cam.intrinsics(1);
fy = cam.intrinsics(2);
cx = cam.intrinsics(3);
cy = cam.intrinsics(4);

% camK = [fx, 0, cx; 0, fy, cy; 0, 0, 1].';
camD = cam.distortion_coeffs;

% params = cameraParameters("IntrinsicMatrix", camK, "RadialDistortion", camD(1:2), ...
%                           "TangentialDistortion", camD(3:4));
% 
% pt_out = undistortPoints(uv_dist, params);

iters = 5;

x = (uv_dist(1) - cx) / fx;
y = (uv_dist(2) - cy) / fy;
x0 = x;
y0 = y;

for i = 1:1:iters
    r2 = x*x + y*y;
    icdist = (1 + ((camD(4)*r2 + camD(3))*r2 + camD(2))*r2)/(1 + ((camD(1)*r2 + fy)*r2 + fx)*r2);
    deltaX = 2*cx*x*y + cy*(r2 + 2*x*x);
    deltaY = cx*(r2 + 2*y*y) + 2*cy*x*y;
    x = (x0 - deltaX)*icdist;
    y = (y0 - deltaY)*icdist;
end

pt_out = [x; y];

end

