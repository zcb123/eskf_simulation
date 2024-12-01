function updateGroundEffect()

    global control_status terrain_vpos;
    GNDEFFECT_TIMEOUT = 10e6;
    if (control_status.flags.in_air && ~control_status.flags.fixed_wing) 
		if (isTerrainEstimateValid()) 
			% automatically set ground effect if terrain is valid
			height = terrain_vpos - states.pos(3);
			control_status.flags.gnd_effect = (height < params.gnd_effect_max_hgt);

        elseif (control_status.flags.gnd_effect) 
			% Turn off ground effect compensation if it times out
			if (isTimedOut(time_last_gnd_effect_on, GNDEFFECT_TIMEOUT)) 
				control_status.flags.gnd_effect = false;
			end
		end

	else 
		control_status.flags.gnd_effect = false;
	end



end

