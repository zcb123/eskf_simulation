function checkVerticalAccelerationHealth()

    global params control_status fault_status;
    global imu_sample_delayed dt_ekf_avg time_last_imu; 
    global vert_pos_fuse_attempt_time_us vert_vel_fuse_time_us vert_pos_innov_ratio vert_vel_innov_ratio;
    global clip_counter time_bad_vert_accel time_good_vert_accel;

    BADACC_PROBATION = 10e6;

    is_inertial_nav_falling = false;
	are_vertical_pos_and_vel_independant = false;

	if (isRecent(vert_pos_fuse_attempt_time_us, 1000000)) 
		if (isRecent(vert_vel_fuse_time_us, 1000000)) 
			% If vertical position and velocity come from independent sensors then we can
			% trust them more if they disagree with the IMU, but need to check that they agree
			using_gps_for_both = control_status.flags.gps_hgt && control_status.flags.gps;
			using_ev_for_both = control_status.flags.ev_hgt && control_status.flags.ev_vel;
			are_vertical_pos_and_vel_independant = ~(using_gps_for_both || using_ev_for_both);
			is_inertial_nav_falling = bitor(is_inertial_nav_falling,(vert_vel_innov_ratio > params.vert_innov_test_lim) && vert_pos_innov_ratio > params.vert_innov_test_min);
			is_inertial_nav_falling = bitor(is_inertial_nav_falling,(vert_pos_innov_ratio > params.vert_innov_test_lim) && vert_vel_innov_ratio > params.vert_innov_test_min);

		else 
			% only height sensing available
			is_inertial_nav_falling = vert_pos_innov_ratio > params.vert_innov_test_lim;
		end
	end

	% Check for more than 50% clipping affected IMU samples within the past 1 second
	clip_count_limit = 1 / dt_ekf_avg;
	is_clipping = imu_sample_delayed.delta_vel_clipping(1) ||...
				 imu_sample_delayed.delta_vel_clipping(2) ||...
				 imu_sample_delayed.delta_vel_clipping(3);

	if (is_clipping && clip_counter < clip_count_limit) 
		clip_counter = clip_counter + 1;

    elseif (clip_counter > 0) 
		clip_counter = clip_counter - 1;
	end

	fault_status.flags.bad_acc_clipping = clip_counter > clip_count_limit / 2;

	is_clipping_frequently = clip_counter > 0;

	% if vertical velocity and position are independent and agree, then do not require evidence of clipping if
	% innovations are large
	bad_vert_accel = (are_vertical_pos_and_vel_independant || is_clipping_frequently) && is_inertial_nav_falling;

	if (bad_vert_accel) 
		time_bad_vert_accel = time_last_imu;

	else 
		time_good_vert_accel = time_last_imu;
	end

	% declare a bad vertical acceleration measurement and make the declaration persist
	% for a minimum of BADACC_PROBATION seconds 
	if (fault_status.flags.bad_acc_vertical) 
		fault_status.flags.bad_acc_vertical = isRecent(time_bad_vert_accel, BADACC_PROBATION);

	else 
		fault_status.flags.bad_acc_vertical = bad_vert_accel;
	end
end


