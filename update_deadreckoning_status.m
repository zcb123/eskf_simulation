function update_deadreckoning_status()
    
    global control_status params time_last_hor_pos_fuse time_last_hor_vel_fuse;
    velPosAiding = (control_status.flags.gps || control_status.flags.ev_pos || control_status.flags.ev_vel) ...
				  && (isRecent(time_last_hor_pos_fuse, params.no_aid_timeout_max) ...
				      || isRecent(time_last_hor_vel_fuse, params.no_aid_timeout_max));

    control_status.flags.inertial_dead_reckoning = ~velPosAiding;

end

