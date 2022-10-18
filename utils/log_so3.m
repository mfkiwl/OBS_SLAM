function omega = log_so3(R)
% =========================================================================
% This function is meant to SO(3) matrix log
% -------------------------------------------------------------------------
% Inputs :
%    R   : a 3x3 matrix
% Outputs:
%  omega : a 3x1 vector
% -------------------------------------------------------------------------
% Copyright (C) 2022 @Yanyu Zhang, yzhan831@ucr.edu
% Copyright (C) 2022 @Jie Xu, jxu150@ucr.edu
% Copyright (C) 2022 @Wei Ren, ren@ece.ucr.edu
% =========================================================================
R11 = R(1, 1);
R12 = R(1, 2);
R13 = R(1, 3);
R21 = R(2, 1);
R22 = R(2, 2);
R23 = R(2, 3);
R31 = R(3, 1);
R32 = R(3, 2);
R33 = R(3, 3);

tr = trace(R);

if (tr + 1.0 < 1e-10)
    if (abs(R33 + 1.0) > 1e-5)
        omega = (pi / sqrt(2.0 + 2.0 * R33)) * [R13; R23; 1.0 + R33];
    elseif (abs(R22 + 1.0) > 1e-5)
        omega = (pi / sqrt(2.0 + 2.0 * R22)) * [R12; 1.0 + R22; R32];
    else
        omega = (pi / sqrt(2.0 + 2.0 * R11)) * [1.0 + R11; R21; R31];
    end
else
    tr_3 = tr - 3.0;
    if (tr_3 < -1e-7)
        theta = acos((tr - 1.0) / 2.0);
        magnitude = theta / (2.0 * sin(theta));
    else
        magnitude = 0.5 - tr_3 / 12.0;
    end
    omega = magnitude * [R32 - R23; R13 - R31; R21 - R12];
end

end
