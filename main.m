% =========================================================================
% This script is used to test
% -------------------------------------------------------------------------
% Copyright (C) 2022 @Yanyu Zhang, yzhan831@ucr.edu
% Copyright (C) 2022 @Jie Xu, jxu150@ucr.edu
% Copyright (C) 2022 @Wei Ren, ren@ece.ucr.edu
% =========================================================================
clear all;
close all;
clc;

[data_T, data] = load_simulated_trajectory('udel_gore.txt');

featmap = simulator(data_T);

%% Plot trajectory, trajectory interpolate, and 3D features
for i = 1:1:length(featmap)
    feats(i, 1:3) = featmap(i).p_FinG;
end

figure(1)
plot3(data(:, 2), data(:, 3), data(:, 4));
hold on;
plot3(feats(:,1), feats(:,2), feats(:,3), '.');






