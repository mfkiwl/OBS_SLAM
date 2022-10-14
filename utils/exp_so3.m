function R = exp_so3(w)
% =========================================================================
% This function is meant to SO(3) matrix exponential
% -------------------------------------------------------------------------
% Inputs :
%    w   : a 3x1 vector
% Outputs:
%    R   : a 3x3 matrix
% -------------------------------------------------------------------------
% Copyright (C) 2022 @Yanyu Zhang, yzhan831@ucr.edu
% Copyright (C) 2022 @Jie Xu, jxu150@ucr.edu
% Copyright (C) 2022 @Wei Ren, ren@ece.ucr.edu
% =========================================================================
w_x = skew_x(w);
theta = norm(w);

% Handle small angle values
if (theta < 1e-7)
    A = 1;
    B = 0.5;
else
    A = sin(theta) / theta;
    B = (1 - cos(theta)) / (theta * theta);
end

% compute so(3) rotation
if (theta == 0)
    R = eye(3, 3);
else
    R = eye(3, 3) + A * w_x + B * w_x * w_x;
end

end