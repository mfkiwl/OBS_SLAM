function [imustate, succ] = get_state(poses, desired_time, true_bias)
% =========================================================================
% This function is to get the state at desired time
% -------------------------------------------------------------------------
% Inputs :
%  poses : the trajectory data
% desired_time: current timestamp
% Outputs:

%  found : T/F
% -------------------------------------------------------------------------
% Copyright (C) 2022 @Yanyu Zhang, yzhan831@ucr.edu
% Copyright (C) 2022 @Jie Xu, jxu150@ucr.edu
% Copyright (C) 2022 @Wei Ren, ren@ece.ucr.edu
% =========================================================================

% Set to default state
imustate = zeros(17, 1);
imustate(5) = 1;

[R_GtoI, p_IinG, w_IinI, v_IinG, success_vel] = get_velocity(poses, desired_time);

hist_true_bias_time = true_bias(:, 1);
hist_true_bias_gyro = true_bias(:, 2:4);
hist_true_bias_accel = true_bias(:, 5:7);

% Find the bounding bias values
success_bias = false;
id_loc = 0;

for i = 1:1:(length(hist_true_bias_time)-1)
    if (hist_true_bias_time(i) < desired_time) && (hist_true_bias_time(i + 1) >= desired_time)
        id_loc = i;
        success_bias = true;
        break;
    end
end

if (~success_vel || ~success_bias)
    succ = false;
    return
end

lambda = (desired_time - hist_true_bias_time(id_loc)) /...
         (hist_true_bias_time(id_loc + 1) - hist_true_bias_time(id_loc));
true_bg_interp = (1 - lambda) * hist_true_bias_gyro(id_loc, :) + lambda * hist_true_bias_gyro(id_loc + 1, :);
true_ba_interp = (1 - lambda) * hist_true_bias_accel(id_loc, :) + lambda * hist_true_bias_accel(id_loc + 1, :);

% Finally lets create the current state
imustate(1, 1) = desired_time;
imustate(2:5, 1) = rot2quat(R_GtoI);
imustate(6:8, 1) = p_IinG;
imustate(9:11, 1) = v_IinG;
imustate(12:14, 1) = true_bg_interp.';
imustate(15:17, 1) = true_ba_interp.';

succ = true;
return

end

