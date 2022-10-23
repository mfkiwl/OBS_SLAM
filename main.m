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
%%
[data_T, data] = load_simulated_trajectory('euroc_V1_01_easy.txt');

% imu measurement: imu.am, imu.wm, imu.timestamp

[featmap, rot, pos] = simulator(data_T);


%% Plot trajectory, trajectory interpolate, and 3D features
for i = 1:1:length(featmap)
    feats(i, 1:3) = featmap(i).p_FinG;
end

plot3(data(:, 2), data(:, 3), data(:, 4));
hold on;
% for i = 1:1:length(pos)
%     plot3(pos(i, 1), pos(i, 2), pos(i, 3), '.');
%     pause(0.1)
% end
plot3(pos(:, 1), pos(:, 2), pos(:, 3), '.');
plot3(feats(:,1), feats(:,2), feats(:,3), '.');

% figure(1)
% xlim([-4 4])
% ylim([-4 4])
% zlim([-4 4])

% for j = 1:1:length(pos)
%     poseplot(rot(3*j-2:3*j,:), pos(j,:), ScaleFactor = 0.1);
%     pause(1)
% end






