function controlFakePosFusion()

    global control_status time_last_fake_pos_fuse using_synthetic_position
    
    fake_pos_data_ready = isTimedOut(time_last_fake_pos_fuse, 2e5); % Fuse fake position at a limited rate

	if (fake_pos_data_ready) 
		continuing_conditions_passing = ~isHorizontalAidingActive();
		starting_conditions_passing = continuing_conditions_passing;

		if (using_synthetic_position) 
			if (continuing_conditions_passing) 
				fuseFakePosition();

				is_fusion_failing = isTimedOut(time_last_fake_pos_fuse, 4e5);

				if (is_fusion_failing) 
					resetFakePosFusion();
				end

			else 
				stopFakePosFusion();
			end

		else 
			if (starting_conditions_passing) 
				startFakePosFusion();

				if (control_status.flags.tilt_align) 
					% The fake position fusion is not started for initial alignement
% 					_warning_events.flags.stopping_navigation = true;
					disp("stopping navigation");
				end
			end
		end
	end

end