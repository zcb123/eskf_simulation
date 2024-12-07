function checkMagInhibition()
 
    global is_yaw_fusion_inhibited mag_inhibit_yaw_reset_req imu_sample_delayed
    is_yaw_fusion_inhibited = shouldInhibitMag();

	if (~is_yaw_fusion_inhibited) 
		mag_use_not_inhibit_us = imu_sample_delayed.time_us;
	end

	% If magnetometer use has been inhibited continuously then a yaw reset is required for a valid heading
	if (imu_sample_delayed.time_us - mag_use_not_inhibit_us > 5e6) 
		mag_inhibit_yaw_reset_req = true;
	end


end

