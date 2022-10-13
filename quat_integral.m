function dy = quat_integral(omega_m,dt)
Omega_mat = [ -matskew(omega_m),  omega_m; -omega_m',            0 ];
dy = expm(0.5*Omega_mat*dt);
end