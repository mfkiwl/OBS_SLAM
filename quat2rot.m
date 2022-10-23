function rot = quat2rot(q)
% =========================================================================
% This function is meant to transfer a JPL quaternion to a rotation matrix
% -------------------------------------------------------------------------
% Inputs :
%    q   : a 4x1 quaternion
% Outputs:
%   rot  : a 3x3 rotation matrix
% -------------------------------------------------------------------------
% Copyright (C) 2022 @Yanyu Zhang, yzhan831@ucr.edu
% Copyright (C) 2022 @Jie Xu, jxu150@ucr.edu
% Copyright (C) 2022 @Wei Ren, ren@ece.ucr.edu
% =========================================================================
q_x = skew_x(q(1:3, 1));
rot = (2 * q(4, 1)^2 - 1) * eye(3, 3) - 2 * q(4, 1) * q_x + ...
    2 * q(1:3, 1) * q(1:3, 1).';
end