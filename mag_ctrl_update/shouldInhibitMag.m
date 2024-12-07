function ret = shouldInhibitMag()

    global params control_status;
    global INDOOR

    user_selected = (params.mag_fusion_type == INDOOR);
	heading_not_required_for_navigation = ~control_status.flags.gps...
			&& ~control_status.flags.ev_pos...
			&& ~control_status.flags.ev_vel;

	ret = (user_selected && heading_not_required_for_navigation)...
	       || isStrongMagneticDisturbance();


end

