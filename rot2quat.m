function q = rot2quat(rot)
% =========================================================================
% This function is meant to transfer a rotation matrix to a JPL quaternion
% -------------------------------------------------------------------------
% Inputs :
%   rot  : a 3x3 rotation matrix
% Outputs:
%    q   : a 4x1 quaternion
% -------------------------------------------------------------------------
% Copyright (C) 2022 @Yanyu Zhang, yzhan831@ucr.edu
% Copyright (C) 2022 @Jie Xu, jxu150@ucr.edu
% Copyright (C) 2022 @Wei Ren, ren@ece.ucr.edu
% =========================================================================
q = zeros(4, 1);
T = trace(rot);

if ((rot(1, 1) >= T) && (rot(1, 1) >= rot(2, 2)) && (rot(1, 1) >= rot(3, 3)))
    q(1) = sqrt((1 + (2 * rot(1, 1)) - T) / 4);
    q(2) = (1 / (4 * q(1))) * (rot(1, 2) + rot(2, 1));
    q(3) = (1 / (4 * q(1))) * (rot(1, 3) + rot(3, 1));
    q(4) = (1 / (4 * q(1))) * (rot(2, 3) - rot(3, 2));
elseif ((rot(2, 2) >= T) && (rot(2, 2) >= rot(1, 1)) && (rot(2, 2) >= rot(3, 3))) 
    q(2) = sqrt((1 + (2 * rot(2, 2)) - T) / 4);
    q(1) = (1 / (4 * q(2))) * (rot(1, 2) + rot(2, 1));
    q(3) = (1 / (4 * q(2))) * (rot(2, 3) + rot(3, 2));
    q(4) = (1 / (4 * q(2))) * (rot(3, 1) - rot(1, 3));
elseif ((rot(3, 3) >= T) && (rot(3, 3) >= rot(1, 1)) && (rot(3, 3) >= rot(2, 2))) 
    q(3) = sqrt((1 + (2 * rot(3, 3)) - T) / 4);
    q(1) = (1 / (4 * q(3))) * (rot(1, 3) + rot(3, 1));
    q(2) = (1 / (4 * q(3))) * (rot(2, 3) + rot(3, 2));
    q(4) = (1 / (4 * q(3))) * (rot(1, 2) - rot(2, 1));
else 
    q(4) = sqrt((1 + T) / 4);
    q(1) = (1 / (4 * q(4))) * (rot(2, 3) - rot(3, 2));
    q(2) = (1 / (4 * q(4))) * (rot(3, 1) - rot(1, 3));
    q(3) = (1 / (4 * q(4))) * (rot(1, 2) - rot(2, 1));
end
  
if (q(4) < 0)
    q = -q;
end
  
q = q / norm(q);

end