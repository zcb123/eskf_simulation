function predictCovariance_Matrix(imu_sampled)

    global states P_M dt_ekf_avg;
    global params control_status;
    q1 = states.quat_nominal(1);
    q2 = states.quat_nominal(2);
    q3 = states.quat_nominal(3);
    q4 = states.quat_nominal(4);

    %delta_v = imu_sample_delayed.delta_vel;

    delta_v = imu_sampled.delta_vel - states.delta_vel_bias;

    Rot = Quat2Tbn([q1 q2 q3 q4]);
    m_R_vb = -Vec2AntiSymMatrix(Rot*delta_v);
    dt = dt_ekf_avg;

    A = [  eye(3)   zeros(3,3) zeros(3,3)   -Rot     zeros(3,3) zeros(3,3) zeros(3,3) zeros(3,2);   %delta_theta
           m_R_vb     eye(3)   zeros(3,3) zeros(3,3)   -Rot     zeros(3,3) zeros(3,3) zeros(3,2);   %delta_vel 
         zeros(3,3) eye(3)*dt   eye(3)    zeros(3,3) zeros(3,3) zeros(3,3) zeros(3,3) zeros(3,2);   %delta_pos 
         zeros(3,3) zeros(3,3) zeros(3,3)   eye(3)   zeros(3,3) zeros(3,3) zeros(3,3) zeros(3,2);   %delta_theta_b 
         zeros(3,3) zeros(3,3) zeros(3,3) zeros(3,3)   eye(3)   zeros(3,3) zeros(3,3) zeros(3,2);   %delta_vel_b 
         zeros(3,3) zeros(3,3) zeros(3,3) zeros(3,3) zeros(3,3)   eye(3)   zeros(3,3) zeros(3,2);   %delta_mag_i
         zeros(3,3) zeros(3,3) zeros(3,3) zeros(3,3) zeros(3,3) zeros(3,3)  eye(3)    zeros(3,2);   %delta_mag_b
         zeros(2,3) zeros(2,3) zeros(2,3) zeros(2,3) zeros(2,3) zeros(2,3) zeros(2,3)   eye(2);];   %delta_wind

    nAng_x = (dt*params.gyro_noise)^2;
    nAng_y = nAng_x;
    nAng_z = nAng_y;
    ang_noise = Rot*diag([nAng_x*nAng_x nAng_y*nAng_y nAng_z*nAng_z])*Rot';     %3x3矩阵

    nVel_x = (dt*params.accel_noise)^2;
    nVel_y = nVel_x;
    nVel_z = nVel_y;
    vel_noise = Rot*diag([nVel_x nVel_y nVel_z])*Rot';                          %3x3矩阵

    d_ang_bias_sig = (dt^2)*params.gyro_bias_p_noise;
    d_vel_bias_sig = (dt^2)*params.accel_bias_p_noise;      
    
    mag_I_sig = params.mage_p_noise^2;
    mag_B_sig = params.magb_p_noise^2;
    %wind_vel_sig = params.wind_vel_p_noise^2;
    wind_vel_sig = 0;
    mag_noise = diag([mag_I_sig mag_I_sig mag_I_sig]);
    magB_noise = diag([mag_B_sig mag_B_sig mag_B_sig]);     %mag_bias_noise
    wind_noise = diag([wind_vel_sig wind_vel_sig]);


    dAngBias = diag([d_ang_bias_sig d_ang_bias_sig d_ang_bias_sig]);
    dvelBias = diag([d_vel_bias_sig d_vel_bias_sig d_vel_bias_sig]);
    
    Q = [ang_noise  zeros(3,3) zeros(3,3) zeros(3,3) zeros(3,3) zeros(3,3) zeros(3,3) zeros(3,2);
         zeros(3,3) vel_noise  zeros(3,3) zeros(3,3) zeros(3,3) zeros(3,3) zeros(3,3) zeros(3,2);
         zeros(3,3) zeros(3,3) zeros(3,3) zeros(3,3) zeros(3,3) zeros(3,3) zeros(3,3) zeros(3,2);
         zeros(3,3) zeros(3,3) zeros(3,3)  dAngBias  zeros(3,3) zeros(3,3) zeros(3,3) zeros(3,2);
         zeros(3,3) zeros(3,3) zeros(3,3) zeros(3,3)  dvelBias  zeros(3,3) zeros(3,3) zeros(3,2);
         zeros(3,3) zeros(3,3) zeros(3,3) zeros(3,3) zeros(3,3) mag_noise  zeros(3,3) zeros(3,2);
         zeros(3,3) zeros(3,3) zeros(3,3) zeros(3,3) zeros(3,3) zeros(3,3) magB_noise zeros(3,2);
         zeros(2,3) zeros(2,3) zeros(2,3) zeros(2,3) zeros(2,3) zeros(2,3) zeros(2,3) wind_noise;];
    
    %tmp = A*P_M*A'
    P_M = A*P_M*A' + Q;
    
    


    P_M = fixCovarianceErrors(P_M,control_status,false);
    
end