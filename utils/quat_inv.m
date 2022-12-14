function q_inv = quat_inv(q)
% =========================================================================
% This function is meant to inverse a JPL quaternion
% -------------------------------------------------------------------------
% Inputs :
%    q   : a 4x1 quaternion
% Outputs:
%  q_inv : a 4x1 quaternion
% -------------------------------------------------------------------------
% Copyright (C) 2022 @Yanyu Zhang, yzhan831@ucr.edu
% Copyright (C) 2022 @Jie Xu, jxu150@ucr.edu
% Copyright (C) 2022 @Wei Ren, ren@ece.ucr.edu
% =========================================================================
q_inv = zeros(4, 1);
q_inv(1:3, 1) = -q(1:3, 1);
q_inv(4, 1) = q(4, 1);
end