function [imu,mea] = generate_measurements(X,dt,cam_para,rate,sigma_g,sigma_a,sigma_wg,sigma_wa,bg,ba,sigma_i,omega_I_gt,a_I_gt)
% X is the camera state 10*n, n is the total timesteps
% cam_para includes cam_para.T_CtoI, cam_para.intrinsics
% rate is num_imu_measurements/num_cam_measurements
% omega_I_gt and a_I_gt are the imu ground truth, both 3*n
% all bias 3*1

% this section is for testing, comment our when finishing
cam_para.intrinsics = [458.654,457.296,367.215,248.375];
cam_para.T_CtoI = [ 1, 0, 0, -0.05;
    0, 0, -1,  0.00;
    0, 1, 0,  0.00;
    0, 0, 0,  1.00];
cam_para.max_depth = 1.2;
cam_para.min_depth = 0.2;
cam_para.wd = [752,480];

%first get imu measurements by adding noise
sigma_wg_d = sigma_wg*sqrt(dt); % rad/sec^2/sqrt(Hz)
sigma_wa_d = sigma_wa*sqrt(dt); % m/sec^3/sqrt(Hz)

sigma_g_d = sigma_g/sqrt(dt); % rad/sec
sigma_a_d = sigma_a/sqrt(dt); % m/sec^2

imu=[];
bg_k = bg;
ba_k = ba;
for k=1:timesteps
    if(k>1)
        bg_k = bg_k + sigma_wg_d*randn(1);
        ba_k = ba_k + sigma_wa_d*randn(1);
    end
    imu(k).omega_I_m = omega_I_gt(:,k)+bg_k+sigma_g_d*randn(1);
    imu(k).a_I_m = a_I_gt(:,k)+ba_k+sigma_a_d*randn(1);
end

% next get camera measurements by checking [u,v] in range or not, then add
% noise
X_L = [];
lenX = size(X,2);
%define parameters
gen.max_depth = 1.2;
gen.min_depth = 0.2;
gen.cam_wd = [752,480];
for k = length(lenX)
    if mod(k,rate) == 0
        p_IinG = X(5:7,k);
        R_GtoI = quat2rot(X(1:4,k));
        n = randi(4)-1; % define how many points are generated
        while (n)
            p_FinG=generatePoints(p_IinG, R_GtoI, gen, cam_para);
            % skip points that are too close to each other
            if(~isempty(X_L))
                d=sqrt((X_L(1,:)-p_FinG(1)).^2 + (X_L(2,:)-p_FinG(2)).^2 + (X_L(3,:)-p_FinG(1)).^2);
                if min(d)<0.1
                    continue;
                end
            end
            X_L(:,end+1)=p_FinG;
            n=n-1;
        end
    end
end
lenX = size(X,2);
R_ItoC = cam_para.T_CtoI(1:3,1:3)';
p_IinC = -R_ItoC*cam_para.T_CtoI(1:3,4);
mea(lenX)=struct();
for k = 1:lenX
    if mod(k,rate)==0 
        % create the measurements: (convert to I frame)
        p_IinG = X(5:7,k);
        R_GtoI = quat2rot(X(1:4,k));
        
        uvs_dist=[];
        uvs_norm = [];
        inds=[];    
        for j=1:size(X_L,2)
            
            p_FinI=R_GtoI*(X_L(:,j)-p_IinG);
            p_FinC=R_ItoC*p_FinI+p_IinC;
            
            if(p_FinC(3)>cam_para.max_depth||p_FinC(3)<cam_para.min_depth)
                continue;
            end
            
            uv_norm=[p_FinC(1)/p_FinC(3);p_FinC(2)/p_FinC(3)];
            uv_dist=[uv_norm(1)*cam.intrinsics(1)+cam.intrinsics(3);uv_norm(2)*cam.intrinsics(2)+cam.intrinsics(4)];
            
            if(uv_dist(1)<1||uv_dist(1)>cam.wd(1)||uv_dist(2)<1||uv_dist(2)>cam.wd(2))
                continue;
            end
            
            if(~isempty(uvs_dist))
                d=sqrt((uvs_dist(1,:)-uv_dist(1)).^2 + (uvs_dist(2,:)-uv_dist(2)).^2 );
                if min(d)<3 % measurements too close
                   continue; 
                end
            end
            uv_dist(1) = uv_dist(1) +sigma_i*randn;
            uv_dist(2) = uv_dist(2) +sigma_i*randn;
            uvs_norm1=[(uv_dist(1)-cam_para.intrinsics(3))/cam_para.intrinsics(1), (uv_dist(2)-cam_para.intrinsics(4))/cam_para.intrinsics(2)];
            uvs_norm=[uvs_norm;uvs_norm1];
            inds=[inds;j];
        end
            mea(k).measurements=uvs_norm;
            mea(k).descriptors=inds;
    end  
end
end