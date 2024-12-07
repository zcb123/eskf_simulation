function startGpsHgtFusion()
    global control_status states gps_sample_delayed;
    global hgt_sensor_offset gps_alt_ref hgt_sensor_offset
    if (~control_status.flags.gps_hgt) 
		if (control_status.flags.rng_hgt) 
			% swith out of range aid
			% calculate height sensor offset such that current
			% measurement matches our current height estimate
			hgt_sensor_offset = gps_sample_delayed.hgt - gps_alt_ref + states.pos(2);

		else 
			hgt_sensor_offset = 0;
			resetHeightToGps();
		end

		setControlGPSHeight();
	end



end

