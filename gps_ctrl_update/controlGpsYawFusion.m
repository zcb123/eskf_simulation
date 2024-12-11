function controlGpsYawFusion(gps_checks_passing,gps_checks_failing,gps_sample_delayed)

    global params control_status control_status_prev ;
    global time_last_gps_yaw_data time_last_imu time_last_gps_yaw_fuse gps_intermittent
    global mag_yaw_reset_req;
    GPSYAW = 1;
    GPS_MAX_INTERVAL = 5e5;
    if (~(params.fusion_mode && GPSYAW)...
	    || control_status.flags.gps_yaw_fault) 
        control_status.flags.gps_yaw = false;   % 		stopGpsYawFusion();
		return;
    end

    
	is_new_data_available = ~isnan(gps_sample_delayed.yaw);


    persistent nb_gps_yaw_reset_available;
    if isempty(nb_gps_yaw_reset_available)
        nb_gps_yaw_reset_available = 0;
    end

	if (is_new_data_available) %GPS数据可用非nan,则进入下一步，否则停止GPS航向校准 

		continuing_conditions_passing = ~gps_checks_failing; %GPS检查通过

		is_gps_yaw_data_intermittent = ~isRecent(time_last_gps_yaw_data, 2 * GPS_MAX_INTERVAL);

		starting_conditions_passing = continuing_conditions_passing...
				&& control_status.flags.tilt_align...
				&& gps_checks_passing...
				&& ~is_gps_yaw_data_intermittent...
				&& ~gps_intermittent;

		time_last_gps_yaw_data = time_last_imu;
        assignin("base","continuing_conditions_passing",continuing_conditions_passing);
		if (control_status.flags.gps_yaw) 
			if (continuing_conditions_passing) 
                %%%%
                fuseGpsYaw(gps_sample_delayed,params,control_status);                           %出货机天线航向偏置180，碳管机偏置90 
				%%%%
                is_fusion_failing = isTimedOut(time_last_gps_yaw_fuse, params.reset_timeout_max);   %7s之后超时 第一遍总是会超时
				if (is_fusion_failing) 
					if (nb_gps_yaw_reset_available > 0) 
						% Data seems good, attempt a reset
						resetYawToGps();
						if (control_status.flags.in_air) 
							nb_gps_yaw_reset_available = nb_gps_yaw_reset_available - 1;
						end

                    elseif (starting_conditions_passing) 
						% Data seems good, but previous reset did not fix the issue
						% something else must be wrong, declare the sensor faulty and stop the fusion
						control_status.flags.gps_yaw_fault = true;
                        control_status.flags.gps_yaw = false;   %stopGpsYawFusion();
						

					else 
						% A reset did not fix the issue but all the starting checks are not passing
						% This could be a temporary issue, stop the fusion without declaring the sensor faulty
                        control_status.flags.gps_yaw = false;       %stopGpsYawFusion();
						
					end
                    disp("gps yaw fuse failed--timeout");
					% TODO: should we give a new reset credit when the fusion does not fail for some time?
				end

			else 
				% Stop GPS yaw fusion but do not declare it faulty
				
                control_status.flags.gps_yaw = false;       %stopGpsYawFusion();
			end

		else 
			if (starting_conditions_passing) 
				% Try to activate GPS yaw fusion
				startGpsYawFusion();  
                disp('resetYawToGps');
                
				mag_yaw_reset_req = false;
				if (control_status.flags.gps_yaw) 
					nb_gps_yaw_reset_available = 1;
				end
			end
		end

    elseif control_status.flags.gps_yaw && isTimedOut(time_last_gps_yaw_data, params.reset_timeout_max)
		% No yaw data in the message anymore. Stop until it comes back.
		
        control_status.flags.gps_yaw = false;   %stopGpsYawFusion();
	end

	% Before takeoff, we do not want to continue to rely on the current heading
	% if we had to stop the fusion
	if ~control_status.flags.in_air...
	   && ~control_status.flags.gps_yaw...
	    && control_status_prev.flags.gps_yaw

		control_status.flags.yaw_align = false;
	end


end

