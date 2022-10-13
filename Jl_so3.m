function J_left = Jl_so3(w)
theta=norm(w);
if(theta<1e-12)
    J_left = eye(3);
else
    a=w/theta;
    J_left = sin(theta)/theta*eye(3)+(1-sin(theta)/theta)*a*a'+((1-cos(theta))/theta)*matskew(a);
end