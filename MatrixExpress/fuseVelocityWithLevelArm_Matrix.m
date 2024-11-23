function fuseVelocityWithLevelArm_Matrix(pos_offset,innov,innov_gate,obs_var)
    

    global states imu_sample_delayed P_M dt_ekf_avg;
    q1 = states.quat_nominal(1);
    q2 = states.quat_nominal(2);
    q3 = states.quat_nominal(3);
    q4 = states.quat_nominal(4);

    dt = dt_ekf_avg;
    Rot = Quat2Tbn([q1 q2 q3 q4]);
    
    BM = Vec2AntiSymMatrix(pos_offset);
    gyro_unbias = (imu_sample_delayed.delta_ang - states.delta_ang_bias) / dt;

    RBWub = Vec2AntiSymMatrix(Rot*BM*gyro_unbias);

    H_rtk = [RBWub eye(3) zeros(3,3) Rot*BM/dt zeros(3,3) zeros(3,3) zeros(3,3) zeros(3,2)];    %3x23
    
    S = H_rtk*P_M*H_rtk';                   %3x23*23*23*23*3
    S(1,1) = S(1,1) + obs_var(1);
    S(2,2) = S(2,2) + obs_var(2);
    S(3,3) = S(3,3) + obs_var(3);
%     k_vel_id = 1;                           %添加观测噪声,这里是3x3矩阵，不用vel_
%     S(k_vel_id,k_vel_id) = S(k_vel_id,k_vel_id) + obs_var(1);
%     S(k_vel_id+1,k_vel_id+1) = S(k_vel_id+1,k_vel_id+1) + obs_var(2);
%     S(k_vel_id+2,k_vel_id+2) = S(k_vel_id+2,k_vel_id+2) + obs_var(3);

    K = P_M*H_rtk'/(S);                             %23x23*23x3*3x3

    %states_x = 0 + K*(innov - H_rtk*states_x);      %states_x = 0;
    
    P_M = P_M*(eye(23,23) - K*H_rtk);
    %调用fuse函数，一列一列地更新
    for i = 1:3     
        fuse(K(:,i),innov(i));
    end

end