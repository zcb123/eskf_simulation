function ret = resetMagHeading(increase_yaw_var,update_buffer)
    global control_status params;
    global states R_to_earth imu_sample_delayed;
    global mag_counter flt_mag_align_start_time;
    global mag_lpf
 % prevent a reset being performed more than once on the same frame
	if (imu_sample_delayed.time_us == flt_mag_align_start_time) 


		ret = true;
        return;
	end

	% low pass filtered mag required
	if (mag_counter == 0) 
		ret =  false;
        return ;
	end

	mag_init = mag_lpf;

	% calculate the observed yaw angle and yaw variance
	yaw_new = 0;
	yaw_new_variance = 0;

	heading_required_for_navigation = control_status.flags.gps || control_status.flags.ev_pos;

	if ((params.mag_fusion_type <= 2) || ((params.mag_fusion_type == 4) && heading_required_for_navigation)) 

		% rotate the magnetometer measurements into earth frame using a zero yaw angle
		R_to_earth = updateYawInRotMat(0, R_to_earth);

		% the angle of the projection onto the horizontal gives the yaw angle
		mag_earth_pred = R_to_earth * mag_init;
		yaw_new = -atan2(mag_earth_pred(2), mag_earth_pred(1)) + getMagDeclination();

		if (increase_yaw_var) 
			yaw_new_variance = sq(fmaxf(params.mag_heading_noise, 1.0e-2));
        end

    elseif (params.mag_fusion_type == 4) 
		% we are operating temporarily without knowing the earth frame yaw angle
		ret = true;
        return
	else 
		% there is no magnetic yaw observation
		ret = false;
        return
	end

	% update quaternion states and corresponding covarainces
	resetQuatStateYaw(yaw_new, yaw_new_variance, update_buffer);

	% set the earth magnetic field states using the updated rotation
	states.mag_I = R_to_earth * mag_init;

	resetMagCov();

	% record the time for the magnetic field alignment event
	flt_mag_align_start_time = imu_sample_delayed.time_us;

	ret = true;

    end