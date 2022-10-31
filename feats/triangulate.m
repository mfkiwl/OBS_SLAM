function [p_FinG, success] = triangulate(meas1, meas2)
% =========================================================================
% This function is meant to triangulation features based on the first two
% measurements, 
% based on https://www.cs.cmu.edu/~16385/s17/Slides/11.4_Triangulation.pdf
% -------------------------------------------------------------------------
% Inputs :
%  meas1 : first uv measurements
%  meas2 : second uv measurements
% Outputs:
% p_FinG : triangulated p_FinG
% success: T/F
% -------------------------------------------------------------------------
% Copyright (C) 2022 @Yanyu Zhang, yzhan831@ucr.edu
% Copyright (C) 2022 @Jie Xu, jxu150@ucr.edu
% Copyright (C) 2022 @Wei Ren, ren@ece.ucr.edu
% =========================================================================
if norm(meas1.uv - meas2.uv) < 10 % uv displacement check
    success = false;
    return;
end

cam = cam_config();

R_ItoC = cam.T_CtoI(1:3,1:3).';
p_IinC = -R_ItoC * cam.T_CtoI(1:3,4);

R_GtoI1 = meas1.R_GtoI;
P_GtoI1 = - R_GtoI1 * meas1.p_IinG;
R_GtoI2 = meas2.R_GtoI;
P_GtoI2 = - R_GtoI2 * meas2.p_IinG;

R_GtoC1 = R_ItoC * R_GtoI1;
R_GtoC2 = R_ItoC * R_GtoI2;
p_GinC1 = R_ItoC * P_GtoI1 + p_IinC;
p_GinC2 = R_ItoC * P_GtoI2 + p_IinC;

T_GtoC1 = [R_GtoC1, p_GinC1];
T_GtoC2 = [R_GtoC2, p_GinC2];

big_P1 = T_GtoC1;
big_P2 = T_GtoC2;

uv_norm1 = undistort(meas1.uv);
uv_norm2 = undistort(meas2.uv);

A = [uv_norm1(2) * big_P1(3, :) - big_P1(2, :);
     big_P1(1, :) - uv_norm1(1) * big_P1(3, :);
     uv_norm2(2) * big_P2(3, :) - big_P2(2, :);
     big_P2(1, :) - uv_norm2(1) * big_P2(3, :)];

[~, ~, V] = svd(A'*A, 'econ');
p_FinG = V(:,end);
p_FinG = p_FinG / p_FinG(4);
p_FinG = p_FinG(1:3, 1);

success = true;

end
