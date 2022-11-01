function mat = hat_se3(vec)
% =========================================================================
% This function is meant for Hat operator
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
mat = zeros(4);
mat(1:3, 1:3) = skew_x(vec(1:3, 1));
mat(1:3, 4) = vec(4:6, 1);

end