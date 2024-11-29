function ret = shouldResetGpsFusion()


    global time_last_hor_vel_fuse time_last_on_ground_us time_last_hor_pos_fuse;
    global control_status params;
    
    is_reset_required = hasHorizontalAidingTimedOut()...
				       || isTimedOut(time_last_hor_pos_fuse, 2 * params.reset_timeout_max);

	
	is_recent_takeoff_nav_failure = control_status.flags.in_air...
			&& isRecent(time_last_on_ground_us, 30000000)...
			&& isTimedOut(time_last_hor_vel_fuse, params.EKFGSF_reset_delay)...
			&& (time_last_hor_vel_fuse > time_last_on_ground_us);

	is_inflight_nav_failure = control_status.flags.in_air...
					     && isTimedOut(time_last_hor_vel_fuse, params.reset_timeout_max)...
					     && isTimedOut(time_last_hor_pos_fuse, params.reset_timeout_max)...
					     && (time_last_hor_vel_fuse > time_last_on_ground_us)...
					     && (time_last_hor_pos_fuse > time_last_on_ground_us);

	ret = is_reset_required | is_recent_takeoff_nav_failure | is_inflight_nav_failure;



end

