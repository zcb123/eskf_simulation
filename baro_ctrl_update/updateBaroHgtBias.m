function updateBaroHgtBias()
    global params baro_b_est baro_sample_delayed gps_sample_delayed
    global baro_data_ready delta_time_baro_us baro_hgt_faulty  baro_hgt_offset;
    global gps_data_ready gps_intermittent gps_checks_passed NED_origin_initialised gps_alt_ref
    % Baro bias estimation using GPS altitude
	if (baro_data_ready && (delta_time_baro_us ~= 0)) 
		dt = saturation(1e-6*double(delta_time_baro_us), 0, 1);
		baro_b_est.setMaxStateNoise(params.baro_noise);
		baro_b_est.setProcessNoiseStdDev(params.baro_drift_rate);
		baro_b_est.predict(dt);
	end

	if (gps_data_ready && ~gps_intermittent...
	    && gps_checks_passed && NED_origin_initialised...       %NED_origin_initialised在10s后置true进入
	    && ~baro_hgt_faulty) 
		% Use GPS altitude as a reference to compute the baro bias measurement

		baro_bias = (baro_sample_delayed.hgt - baro_hgt_offset) - (gps_sample_delayed.hgt - gps_alt_ref);
		disp('gps_alt_ref')
        disp(gps_alt_ref);
		baro_bias_var = getGpsHeightVariance() + sq(params.baro_noise);
		baro_b_est.fuseBias(baro_bias, baro_bias_var);      %baro_bias_var是常值12.252
	end


end

