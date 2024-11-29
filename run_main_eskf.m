run("init_param_and_data_process.m");
run("reset.m");

global states P;
global control_status;
K4_Display = zeros(23,len_update);
len_gps = length(data.RTK.t);
states_quat_nominal_display=zeros(4,len_update);
states_vel_display=zeros(3,len_update);
states_pos_display=zeros(3,len_update);
gps_index_last = 0;

for i = 1:len_update

    imu_sample_updated.delta_ang = imu_delta_ang(i,:)';
    imu_sample_updated.delta_vel = imu_delta_vel(i,:)';
    imu_sample_updated.delta_vel_dt = imu_vel_dt(i,1);
    imu_sample_updated.delta_ang_dt = imu_ang_dt(i,1);
    imu_sample_updated.delta_ang_clipping = logical([0 0 0]);
    imu_sample_updated.delta_vel_clipping = logical([0 0 0]);
    
    predictCovariance(imu_sample_updated);  
    predictCovariance_Matrix(imu_sample_updated);
    predictState(imu_sample_updated);

    gps_data_ready = false;
    gps_dt = data.RTK.t - imu_delta_t(i,1)*1e6;
    gps_index = find(gps_dt<1e3,1,'last');

    if gps_index_last~=gps_index    %目前都默认gps数据是能用的
        gps_data_ready = true;
        gps_index_last = gps_index;
    end

    if ~control_status.flags.tilt_align
        angle_err_var_vec = [P(1,1) P(2,2) P(3,3)];
        if angle_err_var_vec(1) + angle_err_var_vec(2) <sq(3/57.3)
            control_status.flags.tilt_align = true;
        end
    end

    
    if control_status.flags.in_air
        set_in_air_status(false);
    end



    runYawEKFGSF(imu_sample_updated,data.RTK,gps_data_ready,gps_index);

    controlGpsFusion(data.RTK,gps_data_ready,gps_index);

%     K4_Display(:,i) = Kfusion4;
%     states_quat_nominal_display(:,i) = states.quat_nominal;
%     states_vel_display(:,i) = states.vel;
%     states_pos_display(:,i) = states.pos;

end