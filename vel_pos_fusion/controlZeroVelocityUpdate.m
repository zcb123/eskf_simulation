function controlZeroVelocityUpdate()
    global states P k_vel_id control_status control_status_prev;
    global time_last_zero_velocity_fuse time_last_imu
    % Fuse zero velocity at a limited rate (every 200 milliseconds)
	zero_velocity_update_data_ready = isTimedOut(time_last_zero_velocity_fuse, 2e5);

	if (zero_velocity_update_data_ready) 
        %vehicle_at_rest 只有在飞行器静止，加计和陀螺仪变化很小时才会置true
	    continuing_conditions_passing = control_status.flags.vehicle_at_rest ...
				&& control_status_prev.flags.vehicle_at_rest;
				
%         continuing_conditions_passing = true;
		if (continuing_conditions_passing)  
			vel_obs = [0 0 0]';
			innovation = states.vel - vel_obs;

			% Set a low variance initially for faster leveling and higher
			% later to let the states follow the measurements
            if control_status.flags.tilt_align
                obs_var = sq(0.2);
            else
			    obs_var =sq(0.001);
            end
            innov_var1 = P(k_vel_id, k_vel_id)+obs_var;
            innov_var2 = P(k_vel_id+1, k_vel_id+1)+obs_var;
            innov_var3 = P(k_vel_id+2, k_vel_id+2)+obs_var;
			
				
			fuseVelPosHeight(innovation(1), innov_var1, 1);
			fuseVelPosHeight(innovation(2), innov_var2, 2);
			fuseVelPosHeight(innovation(3), innov_var3, 3);

			time_last_zero_velocity_fuse = time_last_imu;
		end
	end



end
