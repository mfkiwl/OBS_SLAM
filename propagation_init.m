%% Propagation Setup

cur_id = 0;

% State Vector
% X = [q_G_I^T p_I_G^T v_I_G^T b_g^T b_a^T | f1_G^T ... fn_G^T]^T
% pose = [q_G_I p_I_G] = [qx qy qz qw X Y Z]
imu = zeros(16, 1);
imu(4) = 1;
cur_id = cur_id + size(imu, 1);

cov = 1e-6 * eye(cur_id, cur_id);