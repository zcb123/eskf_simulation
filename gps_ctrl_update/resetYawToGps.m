function ret = resetYawToGps()
    global params R_to_earth gps_sample_delayed gps_yaw_offset;
    global time_last_imu yaw_signed_test_ratio_lpf time_last_gps_yaw_fuse;
    % define the predicted antenna array vector and rotate into earth frame
    
	ant_vec_bf = [cos(gps_yaw_offset) sin(gps_yaw_offset) 0]';
	ant_vec_ef = R_to_earth * ant_vec_bf;

	% check if antenna array vector is within 30 degrees of vertical and therefore unable to provide a reliable heading
	if (fabsf(ant_vec_ef(3)) > cos((30.0/57.3)))  
        ret = false;
		return ;
	end

	% GPS yaw measurement is alreday compensated for antenna offset in the driver
	measured_yaw = gps_sample_delayed.yaw;

	yaw_variance = sq(fmaxf(params.gps_heading_noise, 1.0e-2));
	resetQuatStateYaw(measured_yaw, yaw_variance, true);

	time_last_gps_yaw_fuse = time_last_imu;

	yaw_signed_test_ratio_lpf.reset_filter(0);
    

	ret = true;

end