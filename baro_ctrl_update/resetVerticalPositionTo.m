function resetVerticalPositionTo(new_vert_pos)

    global states output_buffer output_new;
    global time_last_hgt_fuse time_last_imu state_reset_status;
    old_vert_pos = states.pos(3);
	states.pos(3) = new_vert_pos;

	% store the reset amount and time to be published
	state_reset_status.posD_change = new_vert_pos - old_vert_pos;
	state_reset_status.posD_counter = state_reset_status.posD_counter + 1;

	% apply the change in height / height rate to our newest height / height rate estimate
	% which have already been taken out from the output buffer
	output_new.pos(3) = output_new.pos(3) + state_reset_status.posD_change;

	% add the reset amount to the output observer buffered data
	for i = 1: 3
		output_buffer(i).pos(3) = output_buffer(i).pos(3) + state_reset_status.posD_change;
		%output_vert_buffer[i].vert_vel_integ += state_reset_status.posD_change;
    end

	% add the reset amount to the output observer vertical position state
	%output_vert_new.vert_vel_integ = states.pos(3);

	% Reset the timout timer
	time_last_hgt_fuse = time_last_imu;


end

