function  setGpsData(gps)

global params dt_ekf_avg;
global gps_sample_delayed gps_buffer obs_buffer_length;
global gps_hgt_accurate gps_yaw_offset min_obs_interval_us time_last_gps;
global pos_ref;

global initialised 
if ~initialised
    return
end

% 直接在global_init里边将gps_buffer初始化，这里不额外做初始化了
if gps_buffer.len < 1

    gps_buffer = ring_buffer(obs_buffer_length);

end

    if ((gps.time_us - time_last_gps) > min_obs_interval_us)        
        time_last_gps = gps.time_us;
		gps_sample_new.time_us = gps.time_us - (params.gps_delay_ms * 1000);
		gps_sample_new.time_us = gps_sample_new.time_us - dt_ekf_avg * 5e5; % seconds to microseconds divided by 2
		gps_sample_new.vel = gps.vel_ned;
		gps_speed_valid = gps.vel_ned_valid;
		gps_sample_new.sacc = gps.sacc;
		gps_sample_new.hacc = gps.eph;
		gps_sample_new.vacc = gps.epv;
		gps_sample_new.hgt = double(gps.alt) * 1e-3;			%原始的gps_alt单位是毫米
		gps_sample_new.yaw = gps.yaw;
		gps_sample_new.fix_type = gps.fix_type;

		if ~isnan(gps.yaw_offset) 
			gps_yaw_offset = gps.yaw_offset;
		else 
			gps_yaw_offset = 0.0;
		end

		% Only calculate the relative position if the WGS-84 location of the origin is set
		if (collect_gps(gps)) %10s后返回位置
			gps_sample_new.pos = pos_ref.project((double(gps.lat) / 1.0e7), (double(gps.lon) / 1.0e7));
		else 
			gps_sample_new.pos(1,1) = 0;
			gps_sample_new.pos(2,1) = 0;
		end

		gps_buffer.push(gps_sample_new);

    else
        disp('gps data invalid');
    end   

end
