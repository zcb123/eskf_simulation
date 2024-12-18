function controlFakePosFusion()

    global control_status time_last_fake_pos_fuse using_synthetic_position
    
    fake_pos_data_ready = isTimedOut(time_last_fake_pos_fuse, 2e5); % Fuse fake position at a limited rate 200ms

	if (fake_pos_data_ready)                                            %每隔200ms都会进入        第一次必进
		continuing_conditions_passing = ~isHorizontalAidingActive();    %没有其他的水平校正源则会运行下面的更新
		starting_conditions_passing = continuing_conditions_passing;

		if (using_synthetic_position)                                   %在startFakePosFusion中置true，stopFakePosFusion中置false
			if (continuing_conditions_passing) 
			    fuseFakePosition();                                         %只修水平位置，不修高度
				is_fusion_failing = isTimedOut(time_last_fake_pos_fuse, 4e5);
				if (is_fusion_failing) 
					resetFakePosFusion();
                    disp('resetFakePosFusion();')
				end

			else 
				stopFakePosFusion();                    %有其他水平校正源的时候停止校正
			end

		else 
			if (starting_conditions_passing) 
				startFakePosFusion();                   %time_last_fake_pos_fuse在这里面赋过值

				if (control_status.flags.tilt_align) 
					% The fake position fusion is not started for initial alignement
% 					_warning_events.flags.stopping_navigation = true;
					disp("stopping navigation");
				end
			end
		end
	end

end