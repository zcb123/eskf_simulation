function updated = setSensorData(i,gyro_filted,acc_filted,vehicle_t,vehicle_dt,data)
    
    global params;
    % persistent index ;
    % if isEmpty(index)
    %    index = 1; 
    % end
    % persistent rtk_index
    % if isEmpty(rtk_index)
    %     rtk_index = 1;
    % end
    persistent rtk_index_last
    if isempty(rtk_index_last)
        rtk_index_last = 1;
    end
    % persistent mag_index
    % if isEmpty(mag_index)
    %     mag_index = 1;
    % end
    persistent mag_index_last
    if isempty(mag_index_last)
        mag_index_last = 1;
    end
    % persistent baro_index
    % if isEmpty(baro_index)
    %     baro_index = 1;
    % end
    persistent baro_index_last
    if isempty(baro_index_last)
        baro_index_last = 1;
    end


    imu_sample_new.time_us = uint64(vehicle_t(i,1)*1e6);   %转化成微秒
    imu_sample_new.delta_ang = gyro_filted(i,:)'*vehicle_dt(i,1);
    imu_sample_new.delta_vel = acc_filted(i,:)'*vehicle_dt(i,1);
    imu_sample_new.delta_ang_dt = vehicle_dt(i,1);
    imu_sample_new.delta_vel_dt = vehicle_dt(i,1);
    imu_sample_new.delta_vel_clipping = logical([0 0 0]);
    updated = setIMUData(imu_sample_new);

    rtk_dt = data.RTK.t - vehicle_t(i,1)*1e6;
    rtk_index = find(rtk_dt<1e5,1,'last');
    if rtk_index_last ~= rtk_index

        rtk_index_last = rtk_index;
		gps_msg.time_us = data.RTK.t(rtk_index,1);
		gps_msg.lat = data.RTK.lat(rtk_index,1) ;%日志里已经把系数乘了，这里不再乘
		gps_msg.lon = data.RTK.lon(rtk_index,1) ;
		gps_msg.alt = data.RTK.alt(rtk_index,1) ;
		gps_msg.yaw = data.RTK.hding(rtk_index,1)/57.3;%data.RTK.heading;
		gps_msg.yaw_offset = params.gps_yaw_offset/57.3;
		gps_msg.fix_type = data.RTK.fix(rtk_index,1);
		gps_msg.eph = 0.01;%data.RTK.hstd;
		gps_msg.epv = 0.01;%data.RTK.vstd;
		gps_msg.sacc = 0.01;%0.5;
		
		gps_msg.vel_ned = [
			data.RTK.vN(rtk_index,1)
			data.RTK.vE(rtk_index,1)
			data.RTK.vD(rtk_index,1)
		];

		gps_msg.vel_m_s = norm(gps_msg.vel_ned);
		gps_msg.vel_ned_valid = true;%vehicle_gps_position.vel_ned_valid;
		gps_msg.nsats = data.RTK.svs(rtk_index,1);
		gps_msg.pdop = data.RTK.pdop(rtk_index,1);

        setGpsData(gps_msg);

    end
    
    mag_dt = data.MAG.t - vehicle_t(i,1)*1e6;
    mag_index = find(mag_dt<1e3,1,'last');
    if mag_index_last ~= mag_index
        mag_index_last = mag_index;
		magSample.time_us = data.MAG.t(mag_index);
        mag_bx = double(data.MAG.Y(mag_index,1));    %包含坐标系旋转
        mag_by = double(data.MAG.X(mag_index,1));
        mag_bz = double(data.MAG.Z(mag_index,1));
        magSample.mag = [mag_bx*0.003 mag_by*0.003 mag_bz*0.003]';  

        setMagData(magSample);

    end

    baro_dt = data.BAR0.t - vehicle_t(i,1)*1e6;
    baro_index = find(baro_dt<1e3,1,'last');
    if baro_index_last ~= baro_index
        baro_index_last = baro_index;
        baro.time_us = data.BAR0.t(baro_index,1);
        baro.hgt = data.BAR0.Hight(baro_index,1);
        setBaroData(baro);
    end

end


