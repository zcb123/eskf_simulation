function ret = collect_gps(gps)

    global states filter_initialised 
    global gps_checks_passed NED_origin_initialised gps_alt_ref gps_acc_changed gps_prev;
    global mag_declination_gps mag_inclination_gps mag_strength_gps mag_yaw_reset_req;
    global pos_ref ;
    global time_last_imu;   %这个变量在setIMUData里边赋值
    global earth_rate_NED;
    global NONE;

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

        earth_rate_NED = calcEarthRateNED(radians(pos_ref.getProjectionReferenceLat()));    %暂时没用

        declination_was_valid = ~isnan(mag_declination_gps);
        mag_declination_gps = 0.5;      %这里有复杂的计算      目前没用到
        mag_inclination_gps = 1.5;
        mag_strength_gps = 1.5;         %这里要查表
        
        if ((params.mag_fusion_type ~= NONE)...
		     && ~declination_was_valid) 
			mag_yaw_reset_req = true;
        end
        
        disp("GPS checks passed");
        
    elseif (~NED_origin_initialised) 
%         disp("NED origin not initialised")

        if ((gps.fix_type >= 2) && (gps.eph < 1000)) 

			declination_was_valid = ~isnan(mag_declination_gps);

			% If we have good GPS data set the origin's WGS-84 position to the last gps fix
			lat = gps.lat * 1.0e-7;
% 			lon = gps.lon * 1.0e-7;         %暂时不用

			% set the magnetic field data returned by the geo library using the current GPS position
			mag_declination_gps =  0.5;      %这里有复杂的计算      目前没用到
			mag_inclination_gps = 1.5;         %这里要查表
			mag_strength_gps = 1.5;         %这里要查表

			% request mag yaw reset if there's a mag declination for the first time
			if (params.mag_fusion_type ~= NONE) 
				if (~declination_was_valid && ~isnan(mag_declination_gps)) 
					mag_yaw_reset_req = true;
				end
			end

			earth_rate_NED = calcEarthRateNED(radians(lat));

		end
    end

    if(NED_origin_initialised) 

		if (gps_prev.fix_type ~= gps.fix_type) && (gps_prev.fix_type >= 3 && gps.fix_type >= 3)
			gps_acc_changed = true;
        end

		gps_prev = gps;
        
    end

    ret = NED_origin_initialised && (gps.fix_type >= 3);

end

