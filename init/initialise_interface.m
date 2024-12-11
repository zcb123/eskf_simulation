function ret = initialise_interface(timestamp)

    global params imu_sample_delayed fault_status;
    global FLT_EPSILON;
    global AUTO HEADING MAG_3D UNUSED INDOOR NONE;
    global BARO GNSS RANGE EV UNKNOWN
    global GPS OF INHIBIT_ACC_BIAS EVPOS EVYAW DRAG ROTATE_EV GPSYAW EVVEL

    global imu_buffer output_buffer obs_buffer_length;

    max_time_delay_ms = max(params.sensor_interval_max_ms, params.auxvel_delay_ms);

	% using baro
	if (params.vdist_sensor_type == BARO) 
		max_time_delay_ms = max(params.baro_delay_ms, max_time_delay_ms);
	end

	% using airspeed
	if (params.arsp_thr > FLT_EPSILON) 
		max_time_delay_ms = max(params.airspeed_delay_ms, max_time_delay_ms);
	end

	% mag mode
	if (params.mag_fusion_type ~= NONE) 
		max_time_delay_ms = max(params.mag_delay_ms, max_time_delay_ms);
	end

	% range aid or range height
	if params.range_aid || (params.vdist_sensor_type == RANGE)
		max_time_delay_ms = max(params.range_delay_ms, max_time_delay_ms);
	end

	if bitand(params.fusion_mode,GPS)
		max_time_delay_ms = max(params.gps_delay_ms, max_time_delay_ms);
	end

	if bitand(params.fusion_mode,OF) 
		max_time_delay_ms = max(params.flow_delay_ms, max_time_delay_ms);
	end
    tmp1 = bitor(EVPOS ,EVYAW);
    tmp2 = bitor(tmp1,EVVEL);
	if bitand(params.fusion_mode,uint8(tmp2))
		max_time_delay_ms = max(params.ev_delay_ms, max_time_delay_ms);
	end

	filter_update_period_ms = params.filter_update_interval_us / 1000;

	% 计算适应最大延迟所需的IMU缓冲区长度，并留出一些抖动余量
	imu_buffer_length = ceil(max_time_delay_ms / filter_update_period_ms);

	% set the observation buffer length to handle the minimum time of arrival between observations in combination
	% with the worst case delay from current time to ekf fusion time
	% allow for worst case 50% extension of the ekf fusion time horizon delay due to timing jitter
	ekf_delay_ms = max_time_delay_ms * 1.5;
	obs_buffer_length = round(ekf_delay_ms / filter_update_period_ms);

	% limit to be no longer than the IMU buffer (we can't process data faster than the EKF prediction rate)
	obs_buffer_length = min(obs_buffer_length, imu_buffer_length);

	disp("EKF max time delay %.1f ms, OBS length %d\n");
    
    imu_buffer = ring_buffer(imu_buffer_length);
    output_buffer = ring_buffer(imu_buffer_length);
    
	if imu_buffer.len < 1 || output_buffer.len<1
% 	    || !_output_vert_buffer.allocate(imu_buffer_length)) 

		disp("IMU and output allocate failed");
		ret = false;
        return
    end

    %将所有元素置零
    imu_data_struct = struct('time_us',uint64(0),...
                            'delta_ang',single([0 0 0]'),...
                            'delta_vel',single([0 0 0]'),...)
                            'delta_ang_dt',single(0),...
                            'delta_vel_dt',single(0),...
                            'delta_ang_clipping',logical([0 0 0]'),...
                            'delta_vel_clipping',logical([0 0 0]'));
    
    for i = 1:imu_buffer.len
        imu_buffer.elements{i,1} = imu_data_struct;           %初始化为零,以防后面调用出错
    end
    
    output = struct('time_us',uint64(0),'quat_nominal',single([1 0 0 0]'),'vel',single([0 0 0]'),'pos',single([0 0 0]'));
    for i = 1:output_buffer.len
        output_buffer.elements{i,1} = output;
    end

	imu_sample_delayed.time_us = timestamp;
	imu_sample_delayed.delta_vel_clipping(1) = false;
	imu_sample_delayed.delta_vel_clipping(2) = false;
	imu_sample_delayed.delta_vel_clipping(3) = false;
    
    fault_status.value = 0;
    ret = true;
end

