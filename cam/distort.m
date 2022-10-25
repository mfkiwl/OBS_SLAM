function pt_out = distort(uv_norm)
% =========================================================================
% This function is meant to distort uv point based on the cameara model
% -------------------------------------------------------------------------
% Inputs :
% uv_norm: undistort uv
% Outputs:
% pt_out : distorted point
% -------------------------------------------------------------------------
% Copyright (C) 2022 @Yanyu Zhang, yzhan831@ucr.edu
% Copyright (C) 2022 @Jie Xu, jxu150@ucr.edu
% Copyright (C) 2022 @Wei Ren, ren@ece.ucr.edu
% =========================================================================
cam = cam_config();
cam_d = [cam.intrinsics(1:4), cam.distortion_coeffs];

r = sqrt(uv_norm(1) * uv_norm(1) + uv_norm(2) * uv_norm(2));
r_2 = r * r;
r_4 = r_2 * r_2;

x1 = uv_norm(1) * (1 + cam_d(5) * r_2 + cam_d(6) * r_4) + 2 * cam_d(7) * ...
    uv_norm(1) * uv_norm(2) + cam_d(8) * (r_2 + 2 * uv_norm(1) * uv_norm(1));
y1 = uv_norm(2) * (1 + cam_d(5) * r_2 + cam_d(6) * r_4) + cam_d(7) * ...
    (r_2 + 2 * uv_norm(2) * uv_norm(2)) + 2 * cam_d(8) * uv_norm(1) * uv_norm(2);

pt_out(1) = cam_d(1) * x1 + cam_d(3);
pt_out(2) = cam_d(2) * y1 + cam_d(4);

end
