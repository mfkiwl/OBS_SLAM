function qt = quat_multiply(q, p)
% =========================================================================
% This function is meant to multiply two 4x1 JPL quaternions
% -------------------------------------------------------------------------
% Inputs :
%    q   : a 4x1 quaternion
%    p   : a 4x1 quaternion
% Outputs:
%   qt   : a 4x1 quaternion
% -------------------------------------------------------------------------
% Copyright (C) 2022 @Yanyu Zhang, yzhan831@ucr.edu
% Copyright (C) 2022 @Jie Xu, jxu150@ucr.edu
% Copyright (C) 2022 @Wei Ren, ren@ece.ucr.edu
% =========================================================================
Qm = zeros(4, 4);
Qm(1:3, 1:3) = q(4, 1) * eye(3, 3) - skew_x(q(1:3, 1));
Qm(1:3, 4) = q(1:3, 1);
Qm(4, 1:3) = -q(1:3, 1).';
Qm(4, 4) = q(4, 1);

qt = Qm * p;

if (qt(4, 1) < 0)
    qt = -qt;
end

qt = qt / norm(qt);

end
