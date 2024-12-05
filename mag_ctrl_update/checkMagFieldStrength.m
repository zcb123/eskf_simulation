function checkMagFieldStrength(mag_sample)
    global params control_status;
    global MAG_3D INDOOR;
    if (params.check_mag_strength ...
	    && ((params.mag_fusion_type <= MAG_3D) || (params.mag_fusion_type == INDOOR && control_status.flags.gps))) 

		if (~isnan(mag_strength_gps)) 
			wmm_gate_size = 0.2; % +/- Gauss
			control_status.flags.mag_field_disturbed = ~isMeasuredMatchingExpected(norm(mag_sample), mag_strength_gps, wmm_gate_size);

		else 
			average_earth_mag_field_strength = 0.45; % Gauss
			average_earth_mag_gate_size = 0.40; % +/- Gauss
			control_status.flags.mag_field_disturbed = ~isMeasuredMatchingExpected(norm(mag_sample), average_earth_mag_field_strength, average_earth_mag_gate_size);
		end

	else 
		control_status.flags.mag_field_disturbed = false;
	end

end