function controlFusionModes()
    global control_status control_status_prev params imu_sample_delayed R_to_earth ang_rate_delayed_raw;
    global baro_buffer gps_buffer baro_sample_delayed gps_sample_delayed;
    global baro_hgt_intermittent delta_time_baro_us time_last_baro baro_data_ready;
    global gps_intermittent time_prev_gps_us gps_hgt_accurate time_last_gps gps_data_ready;

    control_status_prev = control_status;

    BARO_MAX_INTERVAL = 2e5;
    GPS_MAX_INTERVAL = 5e5;
    if~control_status.flags.tilt_align
        angle_err_var_vec = calcRotVecVariances();
        if ((angle_err_var_vec(1) + angle_err_var_vec(2)) < sq(radians(3.0))) 
            control_status.flags.tilt_align = true;
        end
    
    end


    if ~baro_buffer.isEmpty()
        baro_hgt_intermittent = ~isRecent(time_last_baro, 2 * BARO_MAX_INTERVAL);

		baro_time_prev = baro_sample_delayed.time_us;
		[baro_sample_tmp,baro_data_ready] = baro_buffer.pop_first_older_than(imu_sample_delayed.time_us);
        
		% if we have a new baro sample save the delta time between this sample and the last sample which is
		% used below for baro offset calculations
		if (baro_data_ready && baro_time_prev ~= 0) 
            baro_sample_delayed = baro_sample_tmp;
			delta_time_baro_us = baro_sample_delayed.time_us - baro_time_prev;
		end
        
    end

    if ~gps_buffer.isEmpty()
    
        gps_intermittent = ~isRecent(time_last_gps, 2 * GPS_MAX_INTERVAL);	%不是最新的，即_gps_intermittent=true表示gps数据太老了;5e5:50毫秒

		% check for arrival of new sensor data at the fusion time horizon
		time_prev_gps_us = gps_sample_delayed.time_us;
		[gps_sample_tmp,gps_data_ready] = gps_buffer.pop_first_older_than(imu_sample_delayed.time_us);
		gps_hgt_accurate = (gps_sample_delayed.vacc < params.req_vacc) && (gps_sample_delayed.fix_type == 6);   %这里用上一个时刻的值
		if (gps_data_ready) 
            gps_sample_delayed = gps_sample_tmp;
			% correct velocity for offset relative to IMU
			pos_offset_body = params.gps_pos_body - params.imu_pos_body;
			vel_offset_body = cross(ang_rate_delayed_raw ,pos_offset_body);
			vel_offset_earth = R_to_earth * vel_offset_body;
			gps_sample_delayed.vel = gps_sample_delayed.vel - vel_offset_earth;

			% correct position and height for offset relative to IMU
			pos_offset_earth = R_to_earth * pos_offset_body;
			gps_sample_delayed.pos = gps_sample_delayed.pos - pos_offset_earth(1:2,1);
			gps_sample_delayed.hgt = gps_sample_delayed.hgt + pos_offset_earth(3);

			gps_sample_delayed.sacc = fmaxf(gps_sample_delayed.sacc, params.gps_vel_noise);

		end
    end




end

