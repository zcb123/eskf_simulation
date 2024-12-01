run("init_param_and_data_process.m");
%%
run("reset_all.m");

global states P R_to_earth;
global control_status filter_initialised imu_sample_delayed;

len_t = length(vehicle_t);
gps_update_time = nan(len_t,2);
mag_update_time = nan(len_t,2);
baro_update_time = nan(len_t,2);
%%
figure('Name','P',Position=[30,450,600,400])
h = surf(P);
shading interp; % 平滑着色
colormap(jet);
colorbar;
xlabel('X 轴');
ylabel('Y 轴');
zlabel('Z 轴');
title('P 动态演示');

figure('Name','R_to_earth',Position=[30,50,600,400])
h_r = surf(R_to_earth);
shading interp; % 平滑着色
colormap(jet);
colorbar;
xlabel('X 轴');
ylabel('Y 轴');
zlabel('Z 轴');
title('R_to_earth 动态演示');
%%
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
        gps_update_time(i,:) = [data.RTK.t(gps_index,1) vehicle_t(i,1)*1e6];
        setGpsData(data.RTK,gps_index);
    end
    
    mag_data_ready = false;
    mag_dt = data.MAG.t - vehicle_t(i,1)*1e6;
    mag_index = find(mag_dt < 1e3,1,'last');
    if mag_index_last ~= mag_index
        mag_data_ready = true;
        mag_index_last = mag_index;
        mag_update_time(i,:) = [data.MAG.t(mag_index,1) vehicle_t(i,1)*1e6];
        setMagData(data.MAG,mag_index);
    end

    baro_data_ready = false;
    baro_dt = data.BAR0.t - vehicle_t(i,1)*1e6;
    baro_index = find(baro_dt < 1e3,1,'last');
    if baro_index_last ~= baro_index
        baro_data_ready = true;
        baro_index_last = baro_index;
        baro_update_time(i,:) = [data.BAR0.t(baro_index,1) vehicle_t(i,1)*1e6];
        setBaroData(data.BAR0,baro_index);
    end
%     baro_data_ready = false;
    if eskf_updated
        if ~filter_initialised
            filter_initialised = initialiseFilter(mag_data_ready);
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

        controlMagFusion(mag_data_ready);
        controlGpsFusion(gps_data_ready); 
        controlHeightFusion(baro_data_ready,gps_data_ready);


    end

    set(h,'ZData',P);
    pause(0.01);
    set(h_r,'ZData',R_to_earth);
    pause(0.01);

    calculateOutputStates(imu_sample_new,eskf_updated);
 
end