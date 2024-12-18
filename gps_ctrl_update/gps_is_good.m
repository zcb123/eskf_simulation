function ret = gps_is_good(gps)

    global control_status params gps_check_fail_status states;
    global last_gps_fail_us last_gps_pass_us time_last_imu;
    global gps_pos_prev gps_alt_prev gps_pos_deriv_filt gps_velNE_filt gps_velD_diff_filt;
    % Check the fix type
	gps_check_fail_status.flags.fix = (gps.fix_type < 3);

	% Check the number of satellites
	gps_check_fail_status.flags.nsats = (gps.nsats < params.req_nsats);

	% Check the position dilution of precision
	gps_check_fail_status.flags.pdop = (gps.pdop > params.req_pdop);

	% Check the reported horizontal and vertical position accuracy
	gps_check_fail_status.flags.hacc = (gps.eph > params.req_hacc);
	gps_check_fail_status.flags.vacc = (gps.epv > params.req_vacc);

	% Check the reported speed accuracy
	gps_check_fail_status.flags.sacc = (gps.sacc > params.req_sacc);

	% check if GPS quality is degraded(退化) 在其他地方用
% 	gps_error_norm = fmaxf((gps.eph / params.req_hacc), (gps.epv / params.req_vacc));
% 	gps_error_norm = fmaxf(gps_error_norm, (gps.sacc / params.req_sacc));

	% Calculate time lapsed since last update, limit to prevent numerical errors and calculate a lowpass filter coefficient
	filt_time_const = 10.0;
	dt = saturation(double(time_last_imu) - double(gps_pos_prev.getProjectionReferenceTimestamp()) * 1e-6, 0.001, filt_time_const);
	filter_coef = dt / filt_time_const;

	% The following checks are only valid when the vehicle is at rest
	lat = double(gps.lat) * 1.0e-7;
	lon = double(gps.lon) * 1.0e-7;

	if (~control_status.flags.in_air && control_status.flags.vehicle_at_rest) 
		% Calculate position movement since last measurement
		delta_pos_n = 0;
		delta_pos_e = 0;

		% calculate position movement since last GPS fix
		if (gps_pos_prev.getProjectionReferenceTimestamp() > 0) 
			gps_pos_prev.project(lat, lon, delta_pos_n, delta_pos_e);

		else 
			% no previous position has been set
			gps_pos_prev.initReference(lat, lon, time_last_imu);
			gps_alt_prev = 1e-3*gps.alt;
		end

		% Calculate the horizontal and vertical drift velocity components and limit to 10x the threshold
		vel_limit = [params.req_hdrift, params.req_hdrift, params.req_vdrift];

		pos_derived = [delta_pos_n, delta_pos_e, (gps_alt_prev - 1e-3 * gps.alt)];
		pos_derived = saturation(pos_derived / dt, -10*vel_limit, 10*vel_limit);

		% Apply a low pass filter
		gps_pos_deriv_filt = pos_derived * filter_coef + gps_pos_deriv_filt * (1 - filter_coef);

		% Calculate the horizontal drift speed and fail if too high
		gps_horizontal_position_drift_rate_m_s = norm(gps_pos_deriv_filt(1:2,1));  % Vector2f(gps_pos_deriv_filt.xy()).norm();
		gps_check_fail_status.flags.hdrift = (gps_horizontal_position_drift_rate_m_s > params.req_hdrift);

		% Fail if the vertical drift speed is too high
		gps_vertical_position_drift_rate_m_s = fabsf(gps_pos_deriv_filt(2));
		gps_check_fail_status.flags.vdrift = (gps_vertical_position_drift_rate_m_s > params.req_vdrift);

		% Check the magnitude of the filtered horizontal GPS velocity
		gps_velNE = saturation(gps.vel_ned(1:2,1),...
					   -10* params.req_hdrift,...
					   10 * params.req_hdrift);
		gps_velNE_filt = gps_velNE * filter_coef + gps_velNE_filt * (1.0 - filter_coef);
		gps_filtered_horizontal_velocity_m_s = gps_velNE_filt.norm();
		gps_check_fail_status.flags.hspeed = (gps_filtered_horizontal_velocity_m_s > params.req_hdrift);

	elseif (control_status.flags.in_air) 
		% These checks are always declared as passed when flying
		% If on ground and moving, the last result before movement commenced is kept
	    gps_check_fail_status.flags.hdrift = false;
	    gps_check_fail_status.flags.vdrift = false;
	    gps_check_fail_status.flags.hspeed = false;

	    resetGpsDriftCheckFilters();

	else 
		% This is the case where the vehicle is on ground and IMU movement is blocking the drift calculation
		resetGpsDriftCheckFilters();
	end

	% save GPS fix for next time
	gps_pos_prev.initReference(lat, lon, time_last_imu);
	gps_alt_prev = 1e-3*gps.alt;

	% Check  the filtered difference between GPS and EKF vertical velocity
	vz_diff_limit = 10.0* params.req_vdrift;
	vertVel = saturation(gps.vel_ned(3) - states.vel(3), -vz_diff_limit, vz_diff_limit);
	gps_velD_diff_filt = vertVel * filter_coef + gps_velD_diff_filt * (1.0- filter_coef);
	gps_check_fail_status.flags.vspeed = (fabsf(gps_velD_diff_filt) > params.req_vdrift);

	% assume failed first time through
	if (last_gps_fail_us == 0) 
		last_gps_fail_us = time_last_imu;
	end

	% if any user selected checks have failed, record the fail time
	global MASK_GPS_NSATS MASK_GPS_PDOP MASK_GPS_HACC MASK_GPS_VACC MASK_GPS_SACC MASK_GPS_HDRIFT MASK_GPS_VDRIFT MASK_GPS_HSPD MASK_GPS_VSPD
	if 	 gps_check_fail_status.flags.fix ||...
		(gps_check_fail_status.flags.nsats   && bitand(params.gps_check_mask , MASK_GPS_NSATS)) ||...
		(gps_check_fail_status.flags.pdop    && bitand(params.gps_check_mask , MASK_GPS_PDOP)) ||...
		(gps_check_fail_status.flags.hacc    && bitand(params.gps_check_mask , MASK_GPS_HACC)) ||...
		(gps_check_fail_status.flags.vacc    && bitand(params.gps_check_mask , MASK_GPS_VACC)) ||...
		(gps_check_fail_status.flags.sacc    && bitand(params.gps_check_mask , MASK_GPS_SACC)) ||...
		(gps_check_fail_status.flags.hdrift  && bitand(params.gps_check_mask , MASK_GPS_HDRIFT)) ||...
		(gps_check_fail_status.flags.vdrift  && bitand(params.gps_check_mask , MASK_GPS_VDRIFT)) ||...
		(gps_check_fail_status.flags.hspeed  && bitand(params.gps_check_mask , MASK_GPS_HSPD)) ||...
		(gps_check_fail_status.flags.vspeed  && bitand(params.gps_check_mask , MASK_GPS_VSPD))
        %以上有一个没通过则gps检查未通过
		last_gps_fail_us = time_last_imu;

	else 
		last_gps_pass_us = time_last_imu;
	end


    ret = isTimedOut(last_gps_fail_us,10000000);        %从第一进入后10s返回true
    
end


