function updateBaroHgtOffset()
    

    global stattes control_status  baro_b_est baro_sample_delayed
    global baro_data_ready delta_time_baro_us baro_hgt_offset
    % calculate a filtered offset between the baro origin and local NED origin if we are not
	% using the baro as a height reference
	if (~control_status.flags.baro_hgt && baro_data_ready && (delta_time_baro_us ~= 0)) 
		local_time_step = saturation(1e-6*delta_time_baro_us, 0, 1);

		% apply a 10 second first order low pass filter to baro offset
		unbiased_baro = baro_sample_delayed.hgt - baro_b_est.getBias();

		offset_rate_correction = 0.1*(unbiased_baro + stattes.pos(3) - baro_hgt_offset);
		baro_hgt_offset = baro_hgt_offset + local_time_step * saturation(offset_rate_correction, -0.1, 0.1);
        
	end


end