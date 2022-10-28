function [H, res] = feat_Jacobian(state, feats)
% =========================================================================
% This function is meant to calculate the measurement Jacobians matrix H
% -------------------------------------------------------------------------
% Inputs :
%  state : current state vector
%  feats : feature vector observed at currrent timestamp
% Outputs:
%    H   : feature Jacobians matrix
%   res  : state residual
% -------------------------------------------------------------------------
% Copyright (C) 2022 @Yanyu Zhang, yzhan831@ucr.edu
% Copyright (C) 2022 @Jie Xu, jxu150@ucr.edu
% Copyright (C) 2022 @Wei Ren, ren@ece.ucr.edu
% =========================================================================
R_GtoI = quat2rot(state(1:4));
p_IinG = state(5:7);

res = zeros(2 * length(feats), 1);

for i = 1:1:length(feats)
    id = feats(i).id;
    feat_pos = 16 + 3 * (id - 1);
    p_finG = feats(i).p_finG;
    p_finI = R_GtoI * (p_finG - p_IinG);

    res(2*i-1 : 2*i, 1) = 

    % Perspective model Jacobains
    H_c = [1/p_finI(3) 0 -p_finI(1)/(p_finI(3) * p_finI(3));
           0 1/p_finI(3) -p_finI(2)/(p_finI(3) * p_finI(3))];

    % State Jacobians
    H_x = zeros(3, length(state));
    H_x(1:3, 1:3) = skew_x(R_GtoI * (p_finG - p_IinG));
    H_x(1:3, 4:6) = -R_GtoI;
    H_x(1:3, feat_pos:(feat_pos+3)) = R_GtoI;

    H((2*i-1):2*i, 1:length(state)) = H_c * H_x;
    
end

end