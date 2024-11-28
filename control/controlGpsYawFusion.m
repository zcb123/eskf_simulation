function controlGpsYawFusion(params,control_status)

%     if (~(params.fusion_mode && GPSYAW)...
% 	    || control_status.flags.gps_yaw_fault) 
% 
% 		stopGpsYawFusion();
% 		return;
% 	end
    global time_last_imu;
% 	is_new_data_available = PX4_ISFINITE(gps_sample_delayed.yaw);

    is_new_data_available = true;

	if (is_new_data_available) 

		continuing_conditions_passing = ~gps_checks_failing;

		is_gps_yaw_data_intermittent = ~isRecent(time_last_gps_yaw_data, 2 * GPS_MAX_INTERVAL);

		starting_conditions_passing = continuing_conditions_passing...
				&& control_status.flags.tilt_align...
				&& gps_checks_passing...
				&& ~is_gps_yaw_data_intermittent...
				&& ~gps_intermittent;

		time_last_gps_yaw_data = time_last_imu;

		if (control_status.flags.gps_yaw) 

			if (continuing_conditions_passing) 

				fuseGpsYaw();

				is_fusion_failing = isTimedOut(time_last_gps_yaw_fuse, params.reset_timeout_max);

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
						stopGpsYawFusion();

					else 
						% A reset did not fix the issue but all the starting checks are not passing
						% This could be a temporary issue, stop the fusion without declaring the sensor faulty
						stopGpsYawFusion();
					end

					% TODO: should we give a new reset credit when the fusion does not fail for some time?
				end

			else 
				% Stop GPS yaw fusion but do not declare it faulty
				stopGpsYawFusion();
			end

		else 
			if (starting_conditions_passing) 
				% Try to activate GPS yaw fusion
				startGpsYawFusion();
				mag_yaw_reset_req = false;
				if (control_status.flags.gps_yaw) 
					nb_gps_yaw_reset_available = 1;
				end
			end
		end

    elseif (control_status.flags.gps_yaw && isTimedOut(time_last_gps_yaw_data, params.reset_timeout_max)) 
		% No yaw data in the message anymore. Stop until it comes back.
		stopGpsYawFusion();
	end

	% Before takeoff, we do not want to continue to rely on the current heading
	% if we had to stop the fusion
% 	if (~control_status.flags.in_air...
% 	    && ~control_status.flags.gps_yaw...
% 	    && control_status_prev.flags.gps_yaw) 
% 		control_status.flags.yaw_align = false;
% 	end


end
