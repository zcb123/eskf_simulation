function resetHorizontalPositionTo(new_horz_pos)

    global states output_buffer output_new;
    global time_last_imu time_last_hor_vel_fuse;

    delta_horz_pos = new_horz_pos - states.pos(1:2,1);
	states.pos(1:2,1) = new_horz_pos;

	for index = 1:3 
		output_buffer(index).pos(1:2,1) = output_buffer(index).pos(1:2,1) + delta_horz_pos;
    end

	output_new.pos(1:2,1) = output_new.pos(1:2,1) + delta_horz_pos;

	%state_reset_status.velNE_change = delta_horz_vel;
	%state_reset_status.velNE_counter ++;

	% Reset the timout timer
	time_last_hor_vel_fuse = time_last_imu;
end