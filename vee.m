function w = vee(w_x)
% =========================================================================
% This function is meant to generate a 3x1 vector from a given 
% skew-symmetric matrix
% -------------------------------------------------------------------------
% Inputs :
%   w_x  : a 3x3 skew-symmetric matrix
% Outputs:
%    w   : a 3x1 vector
% -------------------------------------------------------------------------
% Copyright (C) 2022 @Yanyu Zhang, yzhan831@ucr.edu
% =========================================================================
w = [w_x(3, 2), w_x(1, 3), w_x(2, 1)];
end