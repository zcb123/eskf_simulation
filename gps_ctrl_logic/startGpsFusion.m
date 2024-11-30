function startGpsFusion(gps_sample)

    global control_status;
    if (~control_status.flags.gps) 
		resetHorizontalPositionToGps(gps_sample);

		%when already using another velocity source velocity reset is not necessary
		if (~control_status.flags.opt_flow && ~control_status.flags.ev_vel) 
			resetVelocityToGps(gps_sample);
		end

		%_information_events.flags.starting_gps_fusion = true;
		%ECL_INFO("starting GPS fusion");
        disp("starting GPS fusion");
		control_status.flags.gps = true;
	end
end

