function M = matskew(vector)
v1 = vector(1);
v2 = vector(2);
v3 = vector(3);
M = [0,-v3, v2; v3,0,-v1;-v2,v1,0];
end