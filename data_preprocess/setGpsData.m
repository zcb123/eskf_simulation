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
		gps_sample_new.hgt = gps.alt * 1e-3;			%原始的gps_alt单位是毫米
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
			gps_sample_new.pos(1) = 0;
			gps_sample_new.pos(2) = 0;
		end

		gps_buffer.push(gps_sample_new);

%         gps_sample_delayed.time_us = gps.t(gps_index,1);
%         gps_sample_delayed.lon = gps.lon(gps_index,1);
%         gps_sample_delayed.lat = gps.lat(gps_index,1);
%         gps_sample_delayed.alt = gps.alt(gps_index,1);
%         gps_sample_delayed.pos_ned = [gps.pN(gps_index,1) gps.pE(gps_index,1) gps.pD(gps_index,1)]';
%         gps_sample_delayed.vel_ned = [gps.vN(gps_index,1) gps.vE(gps_index,1) gps.vD(gps_index,1)]';
%         gps_sample_delayed.yaw = gps.hding(gps_index,1);      
%         gps_sample_delayed.hgt = gps.alt(gps_index,1)*1e-3;
%         gps_sample_delayed.hacc = gps.hdop(gps_index,1);        %这个赋值有待商榷
%         gps_sample_delayed.pdop = gps.pdop(gps_index,1);
%         gps_sample_delayed.hdop = gps.hdop(gps_index,1);
%         gps_sample_delayed.sacc = 0.5;
%         gps_sample_delayed.vacc = 0.01;         %代码中0.01
%         gps_sample_delayed.yaw_offset = params.gps_yaw_offset/57.3;
%         gps_sample_delayed.fix_type = gps.fix(gps_index,1);
    
%         gps_hgt_accurate = (gps_sample_delayed.vacc < params.req_vacc) && (gps_sample_delayed.fix_type == 6);
    
%         if ~isnan(gps_sample_delayed.yaw_offset)
%             gps_yaw_offset = gps_sample_delayed.yaw_offset;
%         else
%             gps_yaw_offset = 0;
%         end
%     
%         collect_gps(gps_sample_delayed);
    else
        disp('gps data invalid');
    end   

end
