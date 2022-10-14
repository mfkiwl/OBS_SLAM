function J_right = Jr_so3(w)
% =========================================================================
% This function is meant to computes right Jacobian of SO(3)
% -------------------------------------------------------------------------
% Inputs :
%    w   : a 3x1 vector
% Outputs:
% J_right: a 3x3 matrix
% -------------------------------------------------------------------------
% Copyright (C) 2022 @Jie Xu, jxu150@ucr.edu
% Copyright (C) 2022 @Yanyu Zhang, yzhan831@ucr.edu
% Copyright (C) 2022 @Wei Ren, ren@ece.ucr.edu
% =========================================================================
J_right = Jl_so3(-w);