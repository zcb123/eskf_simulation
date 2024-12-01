function res = getMagDeclination()
    global states control_status params;
    global NED_origin_initialised mag_declination_gps;
    USE_GEO_DECL = 1;
    if (control_status.flags.mag_aligned_in_flight) 
		% Use value consistent with earth field state
		res = atan2(states.mag_I(2), states.mag_I(1));
        return
    elseif (params.mag_declination_source && USE_GEO_DECL) 
		% use parameter value until GPS is available, then use value res =ed by geo library
		if (NED_origin_initialised || ~isnan(mag_declination_gps)) 
			res = mag_declination_gps;
            return
		else 
			res = params.mag_declination_deg / 57.3;
            return
        end

	else 
		% always use the parameter value
		res = params.mag_declination_deg / 57.3;
        return
    end

end


