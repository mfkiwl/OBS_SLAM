function log = log_se3(mat)
% =========================================================================
% This function is meant to SE(3) matrix log
% -------------------------------------------------------------------------
% Inputs :
%   mat  : a 4x4 matrix
% Outputs:
%   log  : a 6x1 vector
% -------------------------------------------------------------------------
% Copyright (C) 2022 @Yanyu Zhang, yzhan831@ucr.edu
% Copyright (C) 2022 @Jie Xu, jxu150@ucr.edu
% Copyright (C) 2022 @Wei Ren, ren@ece.ucr.edu
% =========================================================================
w = log_so3(mat(1:3, 1:3));
T = mat(1:3, 4);
t = norm(w);

if (t < 1e-10)
    log = [w; T];
else
    W = skew_x(w / t);
    Tan = tan(0.5 * t);
    WT = W * T;
    u = T - (0.5 * t) * WT + (1 - t / (2. * Tan)) * (W * WT);
    log = [w; u];
end

end