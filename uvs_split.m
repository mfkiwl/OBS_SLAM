function [uvs_first_two, uvs_all_left] = uvs_split(uvs_all)
% =========================================================================
% This function is meant to figure out the measurements of the common
% features
% -------------------------------------------------------------------------
% Inputs :
% uv_all : a list of all uv measurements
% Outputs:

% -------------------------------------------------------------------------
% Copyright (C) 2022 @Yanyu Zhang, yzhan831@ucr.edu
% Copyright (C) 2022 @Jie Xu, jxu150@ucr.edu
% Copyright (C) 2022 @Wei Ren, ren@ece.ucr.edu
% =========================================================================
all_id = [uvs_all.id].';
all_id_copy = all_id;
uvs_first_two = [];
uvs_all_left = uvs_all;

for i = 1:1:length(all_id)
    found = false;

    for j = 1:1:length(all_id_copy)

        if (~found)
            if (all_id(i) == all_id_copy(j)) && (i ~= j)
                if norm(uvs_all(i).uv - uvs_all(j).uv) >= 10 % uv displacement check
                    uvs_first_two(end + 1, 1).id = uvs_all(i).id;
                    uvs_first_two(end, 1).uv = uvs_all(i).uv;
                    uvs_first_two(end, 1).timestamp = uvs_all(i).timestamp;
                    uvs_first_two(end, 1).p_IinG = uvs_all(i).p_IinG;
                    uvs_first_two(end, 1).R_GtoI = uvs_all(i).R_GtoI;
    
                    uvs_first_two(end, 1).uv2 = uvs_all(j).uv;
                    uvs_first_two(end, 1).timestamp2 = uvs_all(j).timestamp;
                    uvs_first_two(end, 1).p_IinG2 = uvs_all(j).p_IinG;
                    uvs_first_two(end, 1).R_GtoI2 = uvs_all(j).R_GtoI;
    
                    found = true;
                    cur_id = all_id(i);
    
                    % remove the first two measurements from uvs_all
                    uvs_all(i) = [];
                    uvs_all(j) = [];
    
                    all_id(i) = [];
                    all_id(j) = [];
                end
            end
        end

        if found
            all_id_copy = all_id_copy(all_id_copy ~= cur_id);
            break;
        end

    end
end

end