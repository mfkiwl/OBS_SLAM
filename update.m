function [state_n, cov_n] = update(state_c, cov_c, feats)
% =========================================================================
% This function is meant to update the state based on the measurements
% -------------------------------------------------------------------------
% Inputs :
% state_c: current state vector
%  cov_c : current covariance matrix
%  feats : feature vector
% Outputs:
% state_n: next state vector
%  cov_n : next covariance matrix
% -------------------------------------------------------------------------
% Copyright (C) 2022 @Yanyu Zhang, yzhan831@ucr.edu
% Copyright (C) 2022 @Jie Xu, jxu150@ucr.edu
% Copyright (C) 2022 @Wei Ren, ren@ece.ucr.edu
% =========================================================================