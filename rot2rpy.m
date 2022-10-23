function rpy = rot2rpy(rot)

rpy(2, 1) = atan2(-rot(3, 1), sqrt(rot(1, 1) * rot(1, 1) + rot(2, 1) * rot(2, 1)));

if (abs(cos(rpy(2, 1))) > 1.0e-12)
    rpy(3, 1) = atan2(rot(2, 1) / cos(rpy(2, 1)), rot(1, 1) / cos(rpy(2, 1)));
    rpy(1, 1) = atan2(rot(3, 2) / cos(rpy(2, 1)), rot(3, 3) / cos(rpy(2, 1)));
else
    rpy(3, 1) = 0;
    rpy(1, 1) = atan2(rot(1, 2), rot(2, 2));
end

end