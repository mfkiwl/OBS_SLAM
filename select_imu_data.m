function prop_data = select_imu_data(imu, t0, t1)
% =========================================================================
% This function is meant to select IMU reading from t0 to t1
% -------------------------------------------------------------------------
% Inputs :
%   imu  : imu measurement with structure imu(k).am, imu(k).wm,
%   imu(k).timestamp
%   t0   : time t0
%   t1   : time t1
% Outputs:
% prop_data: data used to propagation
% -------------------------------------------------------------------------
% Copyright (C) 2022 @Yanyu Zhang, yzhan831@ucr.edu
% Copyright (C) 2022 @Jie Xu, jxu150@ucr.edu
% Copyright (C) 2022 @Wei Ren, ren@ece.ucr.edu
% =========================================================================
if isempty(imu)
    warning('No IMU measurements detected');
else
    for i = 1:1:(length(imu)-1)
        if (imu(i+1).timestamp > t0) && (imu(i).timestamp < t0)
            data = interpolate_imu(imu.at(i), imu.at(i + 1), time0);
            prop_data(end + 1) = data;
            continue
        end

        if (imu(i).timestamp >= t0) && (imu(i+1).timestamp <= t1)
            prop_data(end + 1) = imu(i);
            continue
        end

        if (imu(i+1).timestamp > t1)
            if (imu(i).timestamp > t1) && (i == 0)
                break
            elseif (imu(i).timestamp > t1)
                data = interpolate_imu(imu.at(i - 1), imu.at(i), time1);
                prop_data(end + 1) = data;
            else
                prop_data(end + 1) = imu(i);
            end

            if (prop_data(length(prop_data)).timestamp ~= t1)
                data = interpolate_imu(imu.at(i), imu.at(i + 1), time1);
                prop_data(end + 1) = data;
            end

            break
        end
    end
end

if isempty(prop_data)
    warning('No IMU measurements detected to propagate');

for i = 1:1:(length(prop_data)-1)
    if (abs(prop_data(i + 1).timestamp - prop_data(i).timestamp) < 1e-12)
        prop_data(i + 1) = [];
        i = i - 1;
    end
end

if (length(prop_data) < 2)
    warning("efk::select_imu_data(): No IMU measurements to propagate " + ...
        "with (%d of 2)", length(prop_data));
end

end


