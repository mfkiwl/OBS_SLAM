function J_left = Jl_so3(w)
% =========================================================================
% This function is meant to computes left Jacobian of SO(3)
% -------------------------------------------------------------------------
% Inputs :
%    w   : a 3x1 vector
% Outputs:
% J_left : a 3x3 matrix
% -------------------------------------------------------------------------
% Copyright (C) 2022 @Jie Xu, jxu150@ucr.edu
% Copyright (C) 2022 @Yanyu Zhang, yzhan831@ucr.edu
% Copyright (C) 2022 @Wei Ren, ren@ece.ucr.edu
% =========================================================================
theta = norm(w);
if(theta < 1e-6)
    J_left = eye(3);
else
    a = w / theta;
    J_left = sin(theta) / theta * eye(3) + (1 - sin(theta) / theta) ...
        * a * a.' + ((1 - cos(theta)) / theta) * skew_x(a);
end