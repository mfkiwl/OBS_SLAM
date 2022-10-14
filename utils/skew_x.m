function w_x = skew_x(w)
% =========================================================================
% This function is meant to generate a skew-symmetric matrix from a given 
% 3x1 vector
% -------------------------------------------------------------------------
% Inputs :
%    w   : a 3x1 vector
% Outputs:
%   w_x  : a 3x3 skew-symmetric matrix
% -------------------------------------------------------------------------
% Copyright (C) 2022 @Yanyu Zhang, yzhan831@ucr.edu
% Copyright (C) 2022 @Jie Xu, jxu150@ucr.edu
% Copyright (C) 2022 @Wei Ren, ren@ece.ucr.edu
% =========================================================================
w_x = [0, -w(3), w(2); w(3), 0, -w(1); -w(2), w(1), 0];
end
