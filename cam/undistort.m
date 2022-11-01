function pt_out = undistort(uv_dist)
% =========================================================================
% This function is meant to undistort uv point based on the cameara model
% -------------------------------------------------------------------------
% Inputs :
% uv_dist: distort uv 1 x 2 matrix 
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

u = uv_dist(1);
v = uv_dist(2);

x = (uv_dist(1) - cx) / fx;
y = (uv_dist(2) - cy) / fy;

x0 = x;
y0 = y;
err = 1;

for i = 1:1:5
    if (err < 0.01)
        break;
    end

    r2 = x*x + y*y;
    icdist = 1 / (1 + (camD(2)*r2 + camD(1))*r2);

    if (icdist < 0)
        x = (u - cx) ./ fx;
        y = (v - cy) ./ fy;
        break;
    end

    deltaX = 2*camD(3)*x*y + camD(4)*(r2 + 2*x*x);
    deltaY = camD(3)*(r2 + 2*y*y) + 2*camD(4)*x*y;
    x = (x0 - deltaX)*icdist;
    y = (y0 - deltaY)*icdist;

    r2 = x*x + y*y;
    r4 = r2 * r2;
    a1 = 2 * x * y;
    a2 = r2 + 2*x*x;
    a3 = r2 + 2*y*y;
    cdist = 1 + camD(1) * r2 + camD(2) * r4;
    xd0 = x * cdist + camD(3)*a1 + camD(4)*a2;
    yd0 = y * cdist + camD(3)*a3 + camD(4)*a1;

    xd = xd0;
    yd = yd0;

    x_proj = xd*fx + cx;
    y_proj = yd*fy + cy;

    err = sqrt((x_proj-u)^2 + (y_proj-v)^2);
end

pt_out = [x; y];

end

