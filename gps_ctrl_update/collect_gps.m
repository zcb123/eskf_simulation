function ret = collect_gps(gps)

    global states filter_initialised 
    global gps_checks_passed NED_origin_initialised gps_alt_ref gps_acc_changed gps_prev;
    global mag_declination_gps mag_strength_gps;
    global pos_ref;
    global time_last_imu;   %这个变量在setIMUData里边赋值
    gps_checks_passed = gps_is_good(gps);

    if (filter_initialised && ~NED_origin_initialised && gps_checks_passed) 
        lat = double(gps.lat) * 1e-7;
		lon = double(gps.lon) * 1e-7;

        if ~pos_ref.isInitialized()

            pos_ref.initReference(lat,lon,time_last_imu);
            
            if isHorizontalAidingActive()            
                [est_lat,est_lon] = pos_ref.reproject(states.pos(1),states.pos(2));
                pos_ref.initReference(est_lat,est_lon,time_last_imu);
            end
        end
        
        gps_alt_ref = 1e-3*double(gps.alt) + states.pos(3);
		NED_origin_initialised = true;

        mag_declination_gps = 0.5;      %这里有复杂的计算
        mag_strength_gps = 1.5;         %这里要查表

        
    elseif (~NED_origin_initialised) 
%         disp("NED origin not initialised")
    end

    if(NED_origin_initialised) 

    
		if((gps_prev.fix_type ~= gps.fix_type) && (gps_prev.fix_type >= 3 && gps.fix_type >= 3)) 
			gps_acc_changed = true;
        end

		gps_prev = gps;
    end

    ret = NED_origin_initialised && (gps.fix_type >= 3);

end

