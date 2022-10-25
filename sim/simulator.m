function featmap_c = simulator(data)
% =========================================================================
% This function is to run the simulation
% -------------------------------------------------------------------------
% Inputs :
%  data  : the trajectory data
% Outputs:
% featmap: a list of features in 3D
% -------------------------------------------------------------------------
% Copyright (C) 2022 @Yanyu Zhang, yzhan831@ucr.edu
% Copyright (C) 2022 @Jie Xu, jxu150@ucr.edu
% Copyright (C) 2022 @Wei Ren, ren@ece.ucr.edu
% =========================================================================
sim = sim_config();
[data_gen, timestamp_start] = feed_trajectory(data);

featmap_n = [];
time_synth = timestamp_start;

% Loop through each pose and generate our feature map in them!!!!
while (true)
    [R_GtoI, p_IinG, succ] = get_pose(data_gen, time_synth);
    featmap_c = featmap_n;

    if (~succ)
        break;
    end
    
    uvs = projectPoints(featmap_c, p_IinG, R_GtoI);
    
    if (length(uvs) < sim.num_pts)
        featmap_n = generatePoints(featmap_c, p_IinG, R_GtoI, sim.num_pts - length(uvs));
        fprintf('[Simalator]: %d features generated at timestamp %f\n', sim.num_pts - length(uvs), time_synth);
    end

    time_synth = time_synth + sim.dt;

end

fprintf('[Simalator]: The total number of feature is %d \n', length(featmap_n));
fprintf('[Simalator]: The total number of timestamp is %d \n', (time_synth-timestamp_start)/sim.dt);
pause(0.1);

end
