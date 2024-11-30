function ret = collect_gps(gps)

    global filter_initialised 
    global NED_origin_initialised gps_alt_ref gps_acc_changed gps_prev;

    gps_checks_passed = gps_is_good(gps);

    if (filter_initialised && ~NED_origin_initialised && gps_checks_passed) 
        gps_alt_ref = 1e-3 * gps.alt + states.pos(2);
		NED_origin_initialised = true;
    elseif (~NED_origin_initialised) 
        disp("NED origin not initialised")
    end

    if(NED_origin_initialised) 

    
		if((gps_prev.fix_type ~= gps.fix_type) && (gps_prev.fix_type >= 3 && gps.fix_type >= 3)) 
			gps_acc_changed = true;
        end

		gps_prev = gps;
    end

    ret = NED_origin_initialised && (gps.fix_type >= 3);

end

