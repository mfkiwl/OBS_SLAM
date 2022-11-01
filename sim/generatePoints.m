function featmap = generatePoints(featmap_c, p_IinG, R_GtoI, nfeats)
% =========================================================================
% This function is meant to generate 3D points
% -------------------------------------------------------------------------
% Inputs :
% featmap_c: current feature map
%  p_IinG: 3 x 1 position of IMU in global frame
%  R_GtoI: 3 x 3 rotational matrix of global frame in IMU
%  nfeats: number of features in each frame
% Outputs:
% featmap: a list of new feature in 3D
% -------------------------------------------------------------------------
% Copyright (C) 2022 @Yanyu Zhang, yzhan831@ucr.edu
% Copyright (C) 2022 @Jie Xu, jxu150@ucr.edu
% Copyright (C) 2022 @Wei Ren, ren@ece.ucr.edu
% =========================================================================
cam = cam_config();
sim = sim_config();

R_CtoI = cam.T_CtoI(1:3,1:3);
p_IinC = -R_CtoI.' * cam.T_CtoI(1:3,4);
w = cam.resolution(1);
h = cam.resolution(2);

featmap = featmap_c;
id_map = length(featmap_c);

% Generate the desired number of features
for i = 1:1:nfeats
    % Uniformly randomly generate within our fov
    u_dist = randi([1 w],1);
    v_dist = randi([1 h],1);

    uv_norm = undistort([u_dist, v_dist]);

    depth = sim.min_depth + (sim.max_depth-sim.min_depth)*rand(1);
    
    bearing = [uv_norm(1); uv_norm(2); 1];
    p_FinC = depth * bearing;

    p_FinI = R_CtoI * (p_FinC - p_IinC);
    p_FinG = R_GtoI.' * p_FinI + p_IinG;
    
    featmap(id_map + i, 1).id_map = id_map + i;
    featmap(id_map + i, 1).p_FinG = p_FinG;

end

end