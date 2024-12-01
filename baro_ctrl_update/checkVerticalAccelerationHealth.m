function checkVerticalAccelerationHealth()


is_inertial_nav_falling = false;
	are_vertical_pos_and_vel_independant = false;

	if (isRecent(_vert_pos_fuse_attempt_time_us, 1000000)) {
		if (isRecent(_vert_vel_fuse_time_us, 1000000)) {
			// If vertical position and velocity come from independent sensors then we can
			// trust them more if they disagree with the IMU, but need to check that they agree
			const using_gps_for_both = _control_status.flags.gps_hgt && _control_status.flags.gps;
			const using_ev_for_both = _control_status.flags.ev_hgt && _control_status.flags.ev_vel;
			are_vertical_pos_and_vel_independant = !(using_gps_for_both || using_ev_for_both);
			is_inertial_nav_falling |= _vert_vel_innov_ratio > _params.vert_innov_test_lim && _vert_pos_innov_ratio > _params.vert_innov_test_min;
			is_inertial_nav_falling |= _vert_pos_innov_ratio > _params.vert_innov_test_lim && _vert_vel_innov_ratio > _params.vert_innov_test_min;

		} else {
			// only height sensing available
			is_inertial_nav_falling = _vert_pos_innov_ratio > _params.vert_innov_test_lim;
		}
	}

	// Check for more than 50% clipping affected IMU samples within the past 1 second
	const uint16_t clip_count_limit = 1.f / _dt_ekf_avg;
	const is_clipping = _imu_sample_delayed.delta_vel_clipping[0] ||
				 _imu_sample_delayed.delta_vel_clipping[1] ||
				 _imu_sample_delayed.delta_vel_clipping[2];

	if (is_clipping && _clip_counter < clip_count_limit) {
		_clip_counter++;

    elseif (_clip_counter > 0) {
		_clip_counter--;
	}

	_fault_status.flags.bad_acc_clipping = _clip_counter > clip_count_limit / 2;

	const is_clipping_frequently = _clip_counter > 0;

	// if vertical velocity and position are independent and agree, then do not require evidence of clipping if
	// innovations are large
	const bad_vert_accel = (are_vertical_pos_and_vel_independant || is_clipping_frequently) && is_inertial_nav_falling;

	if (bad_vert_accel) {
		_time_bad_vert_accel = _time_last_imu;

	} else {
		_time_good_vert_accel = _time_last_imu;
	}

	// declare a bad vertical acceleration measurement and make the declaration persist
	// for a minimum of BADACC_PROBATION seconds
	if (_fault_status.flags.bad_acc_vertical) {
		_fault_status.flags.bad_acc_vertical = isRecent(_time_bad_vert_accel, BADACC_PROBATION);

	} else {
		_fault_status.flags.bad_acc_vertical = bad_vert_accel;
	}


end