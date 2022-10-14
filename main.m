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

data = double(readmatrix('V1_01_easy.txt'));
plot(data(:, 2), data(:, 3));