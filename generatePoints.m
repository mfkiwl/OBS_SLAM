function p_FinG=generatePoints(p_IinG, R_GtoI, gen, cam)
R_ItoC = cam.T_CtoI(1:3,1:3)';
p_IinC = -R_ItoC*cam.T_CtoI(1:3,4);

u_dist = randi([1 gen.cam_wd(1)],1);
v_dist = randi([1 gen.cam_wd(2)],1);
uv_norm=[(u_dist-cam.intrinsics(3))/cam.intrinsics(1); (v_dist-cam.intrinsics(4))/cam.intrinsics(2);1];
depth = gen.min_depth + (gen.max_depth-gen.min_depth)*rand(1);
% depth = 3;
% exter
p_FinC = depth*uv_norm;
p_FinI = R_ItoC'*(p_FinC-p_IinC);
p_FinG = R_GtoI'*p_FinI+p_IinG;
end