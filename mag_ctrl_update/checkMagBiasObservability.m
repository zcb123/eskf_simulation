function checkMagBiasObservability()

    global mag_bias_observable params imu_sample_delayed;
    global yaw_delta_ef time_yaw_started yaw_rate_lpf_ef;
% check if there is enough yaw rotation to make the mag bias states observable
    
	if (~mag_bias_observable && (fabsf(yaw_rate_lpf_ef) > params.mag_yaw_rate_gate)) 
		% initial yaw motion is detected
		mag_bias_observable = true;

    elseif (mag_bias_observable) 
		% require sustained yaw motion of 50% the initial yaw rate threshold
		yaw_dt = 1e-6*(imu_sample_delayed.time_us - time_yaw_started);
		min_yaw_change_req =  0.5 * params.mag_yaw_rate_gate * yaw_dt;
		mag_bias_observable = fabsf(yaw_delta_ef) > min_yaw_change_req;
    end

	yaw_delta_ef = 0.0;
	time_yaw_started = imu_sample_delayed.time_us;



end

