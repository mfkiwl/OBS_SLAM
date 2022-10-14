function q_n = quatnorm(q)
% =========================================================================
% This function is meant to norm a JPL quaternion
% -------------------------------------------------------------------------
% Inputs :
%    q   : a 4x1 quaternion
% Outputs:
%   q_n  : a 4x1 quaternion
% -------------------------------------------------------------------------
% Copyright (C) 2022 @Yanyu Zhang, yzhan831@ucr.edu
% Copyright (C) 2022 @Jie Xu, jxu150@ucr.edu
% Copyright (C) 2022 @Wei Ren, ren@ece.ucr.edu
% =========================================================================
if (q(4, 1) < 0)
    q = -q;
end

q_n = q / norm(q);

end