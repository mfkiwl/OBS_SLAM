function generate_data()
fileID = fopen('data/sim.txt','w');
fprintf(fileID,'%s %s %s %s %s %s %s %s\n','# format: timestamp','tx','ty','tz','qx','qy','qz','qw');
t = 0:0.05:79.95;
x = [0.01:0.01:4, 4*ones(1,400), 4:-0.01:0.01, 0.01*ones(1,400)];
y = [0.01*ones(1,400), 0.01:0.01:4, 4*ones(1,400), 4:-0.01:0.01];
z = zeros(1, 1600);
qx = zeros(1, 1600);
qy = zeros(1, 1600);
qz = [zeros(1,400), sqrt(2)/2, zeros(1,399), sqrt(2)/2, zeros(1,399), sqrt(2)/2, zeros(1,399)];
qw = [zeros(1,400), sqrt(2)/2, zeros(1,399), sqrt(2)/2, zeros(1,399), sqrt(2)/2, zeros(1,399)];
fprintf(fileID,'%f %f %f %f %f %f %f %f\n', [t; x; y; z; qx; qy; qz; qw]);
fclose(fileID);
end

