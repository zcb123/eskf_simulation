function ret = initialiseFilter()

    global states;
    global imu_sample_delayed mag_sample_delayed;
    global is_first_imu_sample;
    global time_last_imu time_last_hgt_fuse time_last_hor_pos_fuse time_last_hor_vel_fuse time_last_hagl_fuse time_last_flow_terrain_fuse time_last_of_fuse;
    global accel_lpf gyro_lpf;
    global mag_counter mag_lpf mag_buffer;
    global baro_buffer baro_sample_delayed baro_hgt_offset baro_counter

    imu_init = imu_sample_delayed;
    
    if (imu_init.delta_vel_dt < 1e-4 || imu_init.delta_ang_dt < 1e-4) 
		ret = false;
        return;
    end

    if is_first_imu_sample
        accel_lpf.reset(imu_init.delta_vel/imu_init.delta_vel_dt);
        gyro_lpf.reset(imu_init.delta_ang/imu_init.delta_ang_dt);
        is_first_imu_sample = false;
    else
        accel_lpf.update(imu_init.delta_vel/imu_init.delta_vel_dt);
        gyro_lpf.update(imu_init.delta_ang/imu_init.delta_ang_dt);
    end
    
    if mag_buffer.len >=1    %磁力计数据初始化成功
        [mag_sample,mag_updated] = mag_buffer.pop_first_older_than(imu_sample_delayed.time_us);
        if mag_updated
            if mag_counter == 0
                mag_lpf.reset(mag_sample);
            else
                mag_lpf.update(mag_sample);    
            end
            mag_counter = mag_counter + 1;
        end
    end

    if baro_buffer.len >=1    %气压计数据初始化成功
        [baro_sample_delayed,bar_updated] = baro_buffer.pop_first_older_than(imu_sample_delayed.time_us);
        if bar_updated
            if baro_counter == 0
               baro_hgt_offset = baro_sample_delayed.hgt;
            else
               baro_hgt_offset = 0.9*baro_hgt_offset + 0.1*baro_sample_delayed.hgt; 
            end
            baro_counter = baro_counter + 1;
        end
    end

    setControlBaroHeight();

    if ~initialiseTilt(accel_lpf,gyro_lpf)
        disp("Initial tilt false\n")
        ret = false;
        return;
    end

    resetMagHeading(false,false);

    initialiseCovariance();

    if(params.mag_fusion_type<=MAG_3D)
        increaseQuatYawErrVariance(sq(fmaxf(params.mag_heading_noise, 1.0e-2)))
    end

    %Initialise the terrain estimator
    %initHagl(); 

    time_last_hgt_fuse = time_last_imu;
	time_last_hor_pos_fuse = time_last_imu;
	time_last_hor_vel_fuse = time_last_imu;
	time_last_hagl_fuse = time_last_imu;
	time_last_flow_terrain_fuse = time_last_imu;
	time_last_of_fuse = time_last_imu;

    alignOutputFilter()
    
    ret = true;
end

