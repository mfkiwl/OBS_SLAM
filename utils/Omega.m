function mat = Omega(w)
% =========================================================================
% This function is meant to integrate quaternion from angular velocity
% -------------------------------------------------------------------------
% Inputs :
%  omega : a 3x1 vector
% Outputs:
%   mat  : a 4x4 matrix
% -------------------------------------------------------------------------
% Copyright (C) 2022 @Yanyu Zhang, yzhan831@ucr.edu
% Copyright (C) 2022 @Jie Xu, jxu150@ucr.edu
% Copyright (C) 2022 @Wei Ren, ren@ece.ucr.edu
% =========================================================================
mat = [-skew_x(w), w; -w.', 0];
end