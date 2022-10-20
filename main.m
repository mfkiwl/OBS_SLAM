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

[data_T, data] = load_simulated_trajectory('V1_01_easy.txt');

% imu measurement: imu.am, imu.wm, imu.timestamp

featmap = simulator(data_T);

for i = 1:1:length(featmap.')
    uv_map(i, 1:3) = featmap(i).p_FinG;
end

plot3(data(:, 2), data(:, 3), data(:, 4));
hold on;
plot3(uv_map(:,1), uv_map(:,2), uv_map(:,2), '.')