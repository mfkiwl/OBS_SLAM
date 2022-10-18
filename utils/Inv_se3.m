function Tinv = Inv_se3(T)
% =========================================================================
% This function is meant to inverse of the se(3)
% -------------------------------------------------------------------------
% Inputs :
%    T   : a 4x4 matrix
% Outputs:
%   Tinv : a 4x4 matrix
% -------------------------------------------------------------------------
% Copyright (C) 2022 @Yanyu Zhang, yzhan831@ucr.edu
% Copyright (C) 2022 @Jie Xu, jxu150@ucr.edu
% Copyright (C) 2022 @Wei Ren, ren@ece.ucr.edu
% =========================================================================
Tinv = eye(4);
Tinv(1:3, 1:3) = T(1:3, 1:3).';
Tinv(1:3, 4) = -Tinv(1:3, 1:3) * T(1:3, 4);
end