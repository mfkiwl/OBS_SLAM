function uvs = projectPoints(uvs_c, feats, p_IinG, R_GtoI)
% =========================================================================
% This function is meant to project 3D points into current frame and
% generate <u, v> measurements
% -------------------------------------------------------------------------
% Inputs :
%  uvs_c : current uvs
%  feats : current feature map
%  p_IinG: 3 x 1 position of IMU in global frame
%  R_GtoI: 3 x 3 rotational matrix of global frame in IMU
% Outputs:
%   uvs  : a list of measurements
% -------------------------------------------------------------------------
% Copyright (C) 2022 @Yanyu Zhang, yzhan831@ucr.edu
% Copyright (C) 2022 @Jie Xu, jxu150@ucr.edu
% Copyright (C) 2022 @Wei Ren, ren@ece.ucr.edu
% =========================================================================
cam = cam_config();
sim = sim_config();

R_ItoC = cam.T_CtoI(1:3,1:3)';
p_IinC = -R_ItoC * cam.T_CtoI(1:3,4);
uvs = uvs_c;

for i = 1:1:length(feats)
    p_FinI = R_GtoI * (feats(i).p_FinG - p_IinG);
    p_FinC = R_ItoC * p_FinI + p_IinC;

    if (p_FinC(3) > sim.max_depth || p_FinC(3) < 0.1)
        continue;
    end

    uv_norm = [p_FinC(1)/p_FinC(3), p_FinC(2)/p_FinC(3)];
    uv_dist = distort(uv_norm);

    if (uv_dist(1) < 0 || uv_dist(1) > cam.resolution(1) || uv_dist(2) < 0 || uv_dist(2) > cam.resolution(2))
        continue;
    end

    uvs(end + 1) = struct('id', feats(i).id_map, 'uv', uv_dist);
end

end



