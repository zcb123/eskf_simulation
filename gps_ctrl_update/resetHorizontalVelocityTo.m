function resetHorizontalVelocityTo(new_horz_vel)

    global states output_buffer output_new;
    global time_last_imu time_last_hor_vel_fuse;
   

    delta_horz_vel = new_horz_vel - states.vel(1:2,1);
	states.vel(1:2,1) = new_horz_vel;

	for index = 1:output_buffer.len 
		output_buffer.elements{index,1}.vel(1:2,1) = output_buffer.elements{index,1}.vel(1:2,1) + delta_horz_vel;
    end

	output_new.vel(1:2,1) = output_new.vel(1:2,1) + delta_horz_vel;

	%state_reset_status.velNE_change = delta_horz_vel;
	%state_reset_status.velNE_counter ++;

	% Reset the timout timer
	time_last_hor_vel_fuse = time_last_imu;




end


