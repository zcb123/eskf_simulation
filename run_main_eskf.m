run("init_param_and_data_process.m");
%%
run("reset_all.m");

global states P;
global control_status filter_initialised imu_sample_delayed;

len_t = length(vehicle_t);

for i = 1:len_t

    imu_sample_new.time_us = uint64(vehicle_t(i,1)*1e6);   %转化成微秒
    imu_sample_new.delta_ang = gyro_filted(i,:)'*vehicle_dt(i,1);
    imu_sample_new.delta_vel = acc_filted(i,:)'*vehicle_dt(i,1);
    imu_sample_new.delta_ang_dt = vehicle_dt(i,1);
    imu_sample_new.delta_vel_dt = vehicle_dt(i,1);
    imu_sample_new.delta_vel_clipping = logical([0 0 0]);

    eskf_updated = setIMUData(imu_sample_new);
    
    gps_data_ready = false;
    gps_dt = data.RTK.t - vehicle_t(i,1)*1e6;
    gps_index = find(gps_dt<1e3,1,'last');
    if gps_index_last~=gps_index    %目前都默认gps数据是能用的
        gps_data_ready = true;
        gps_index_last = gps_index;
        setGpsData(data.RTK,gps_index);
    end
    
    
    if eskf_updated
        if ~filter_initialised
            filter_initialised = initialiseFilter();
            if ~filter_initialised
                disp("eskf initialised failed");
                continue;
            end
        end
        predictCovariance(imu_sample_delayed);
        %predictCovariance_Matrix(imu_sample_delayed);
        predictState(imu_sample_delayed);
        if ~control_status.flags.tilt_align
            angle_err_var_vec = [P(1,1) P(2,2) P(3,3)];
            if angle_err_var_vec(1) + angle_err_var_vec(2) <sq(3/57.3)
                control_status.flags.tilt_align = true;
            end
        end

        runYawEKFGSF(imu_sample_delayed,data.RTK,gps_data_ready,gps_index);
        controlGpsFusion(data.RTK,gps_data_ready,gps_index);
        controlHeightFusion();
    end

    calculateOutputStates(imu_sample_new,eskf_updated);
 
end

%%
% figure
% plot()

%%


K4_Display = zeros(23,len_update);
len_gps = length(data.RTK.t);
states_quat_nominal_display=zeros(4,len_update);
states_vel_display=zeros(3,len_update);
states_pos_display=zeros(3,len_update);