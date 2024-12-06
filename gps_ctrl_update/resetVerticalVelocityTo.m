function resetVerticalVelocityTo(new_vert_vel)

    global states output_buffer output_new;
    global time_last_imu time_last_hor_vel_fuse;
   

    delta_vert_vel = new_vert_vel - states.vel(3,1);
	states.vel(3,1) = new_vert_vel;

	for index = 1:output_buffer.len 
		output_buffer.elements{index,1}.vel(3,1) = output_buffer.elements{index,1}.vel(3,1) + delta_vert_vel;
    end

	output_new.vel(3,1) = output_new.vel(3,1) + delta_vert_vel;

	%state_reset_status.velNE_change = delta_horz_vel;
	%state_reset_status.velNE_counter ++;

	% Reset the timout timer
	time_last_hor_vel_fuse = time_last_imu;

end


