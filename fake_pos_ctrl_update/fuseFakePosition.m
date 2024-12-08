function fuseFakePosition()

    global states params control_status;
    global last_known_posNE time_last_imu time_last_fake_pos_fuse;
    global gps_pos_innov_var gps_pos_test_ratio;
    
    fake_pos_obs_var = zeros(2,1);

	if (control_status.flags.in_air && control_status.flags.tilt_align) 
		fake_pos_obs_var(1) =  sq(fmaxf(params.pos_noaid_noise, params.gps_pos_noise));
        fake_pos_obs_var(2) = fake_pos_obs_var(1);

    elseif (~control_status.flags.in_air && control_status.flags.vehicle_at_rest) 
		% Accelerate tilt fine alignment by fusing more
		% aggressively when the vehicle is at rest
		fake_pos_obs_var(1) =  sq(0.01);
        fake_pos_obs_var(2) = fake_pos_obs_var(1); 

    else 
		fake_pos_obs_var(1) =  sq(0.5);
        fake_pos_obs_var(2) = fake_pos_obs_var(1); 
	end

	gps_pos_innov = states.pos(1:2,1) - last_known_posNE;

	fake_pos_innov_gate = 3;

    [gps_pos_innov_var, gps_pos_test_ratio, ret] = fuseHorizontalPosition(gps_pos_innov, fake_pos_innov_gate, fake_pos_obs_var,true);
	                           
	if (ret) 
		time_last_fake_pos_fuse = time_last_imu;
	end


end

