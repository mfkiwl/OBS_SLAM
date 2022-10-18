function mat = exp_se3(vec)
% =========================================================================
% This function is meant to se(3) matrix exponential
% -------------------------------------------------------------------------
% Inputs :
%   vec  : a 6x1 vector
% Outputs:
%   mat  : a 4x4 matrix
% -------------------------------------------------------------------------
% Copyright (C) 2022 @Yanyu Zhang, yzhan831@ucr.edu
% Copyright (C) 2022 @Jie Xu, jxu150@ucr.edu
% Copyright (C) 2022 @Wei Ren, ren@ece.ucr.edu
% =========================================================================
w = vec(1:3);
u = vec(4:6);
theta = sqrt(dot(w, w));

wskew = skew_x(w);

if (theta < 1e-7)
    A = 1;
    B = 0.5;
    C = 1.0 / 6.0;
else 
    A = sin(theta) / theta;
    B = (1 - cos(theta)) / (theta * theta);
    C = (1 - A) / (theta * theta);
end

V = eye(3) + B * wskew + C * wskew * wskew;

mat = eye(4);
mat(1:3, 1:3) = eye(3) + A * wskew + B * wskew * wskew;
mat(1:3, 4) = V * u;

end
