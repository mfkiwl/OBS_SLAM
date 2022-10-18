function traj_data = load_simulated_trajectory(path)
% =========================================================================
% This function is to load the trajectory data for simulation
% -------------------------------------------------------------------------
% Inputs :
%  path  : the path of the trajectory file
% Outputs:
% traj_data: trajectory data
% -------------------------------------------------------------------------
% Copyright (C) 2022 @Yanyu Zhang, yzhan831@ucr.edu
% Copyright (C) 2022 @Jie Xu, jxu150@ucr.edu
% Copyright (C) 2022 @Wei Ren, ren@ece.ucr.edu
% =========================================================================
traj_data = [];
data = double(readmatrix(path));
for i = 1:1:length(data)
    cur_data = data(i, :);
    rot = quat2rot(cur_data(5:8).');

    single_data.timestamp = data(i, 1);
    single_data.pose = [rot, cur_data(2:4).'; zeros(1, 3), 1];

    traj_data = [traj_data; single_data];
end