function generate_data()
fileID = fopen('sim.txt','w');
fprintf(fileID,'%s %s %s %s %s %s %s %s\n','# format: timestamp','tx','ty','tz','qx','qy','qz','qw');
t = 0:0.05:79.95;
x = [0.00:0.01:3.99, 4*ones(1,400), 3.99:-0.01:0.00, 0.00*ones(1,400)];
y = [0.00*ones(1,400), 0.00:0.01:3.99, 4*ones(1,400), 3.99:-0.01:0.00];
z = zeros(1, 1600);
qx = [zeros(1, 400), -1/2*ones(1,400), -sqrt(2)/2*ones(1,400), 1/2*ones(1,400)];
qy = [sqrt(2)/2*ones(1,400), 1/2*ones(1,400), zeros(1,400), 1/2*ones(1,400)];
qz = [zeros(1,400), 1/2*ones(1,400), sqrt(2)/2*ones(1,400), -1/2*ones(1,400)];
qw = [sqrt(2)/2*ones(1,400), 1/2*ones(1,400), zeros(1,400), 1/2*ones(1,400)];
fprintf(fileID,'%f %f %f %f %f %f %f %f\n', [t; x; y; z; qx; qy; qz; qw]);
fclose(fileID);
end
