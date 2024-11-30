function controlGpsFusion(gps_data,data_ready,gps_index)
    global P;
    global params control_status;
    global gps_sample_delayed;
%     NED_origin_initialised = true;
    global NED_origin_initialised last_gps_pass_us last_gps_fail_us;
    global ekfgsf_yaw_reset_count;
    if data_ready    %目前都默认gps数据是能用的
        
        time_prev_gps_us = gps_sample_delayed.time_us;
        
        gps_checks_passing = isTimedOut(last_gps_fail_us, 5e6);	%上一次last_imu_t - gpsfail时间 > 0.5s
		gps_checks_failing = isTimedOut(last_gps_pass_us, 5e6); %上一次last_imu_t - gpspass时间 > 0.5s
        %%%%
        controlGpsYawFusion(gps_checks_passing,gps_checks_failing,gps_sample_delayed);
        %%%%
        mandatory_conditions_passing = control_status.flags.tilt_align && control_status.flags.yaw_align && NED_origin_initialised;
				
		continuing_conditions_passing = mandatory_conditions_passing && ~gps_checks_failing;
		starting_conditions_passing = continuing_conditions_passing && gps_checks_passing;
		
        if control_status.flags.gps
            if (mandatory_conditions_passing)
                if (continuing_conditions_passing)
                    %%%%
                    fuseGpsVelPos(gps_sample_delayed,params);
                    %%%%
                    if shouldResetGpsFusion()
                        was_gps_signal_lost = isTimedOut(time_prev_gps_us, 1000000);
                        if isYawFailure() && control_status.flags.in_air...
                                && ~was_gps_signal_lost...
                                && ekfgsf_yaw_reset_count < params.EKFGSF_reset_count_limit...
                                && isTimedOut(ekfgsf_yaw_reset_time,5000000)
                            
                            resetYawToEKFGSF();
                        else
                            todo = 1;
                        end
                        %resetVelocityToGps
                        resetVelocityToGps(gps_sample_delayed);
                        resetHorizontalPositionToGps(gps_sample_delayed);

                    end
                else
                    stopGpsFusion();
                    disp("GPS quality poor - stopping use");
                end
            else
                stopGpsFusion();
                disp("GPS bad")
            end
        else
            if starting_conditions_passing

                  startGpsFusion();

            elseif gps_checks_passing&&~control_status.flags.yaw_align && (params.mag_fusion_type == 5) %NONE = 5
                
                if (resetYawToEKFGSF()) 

                    resetVelocityToGps(gps_sample_delayed);
					resetHorizontalPositionToGps(gps_sample_delayed);
                    disp("Yaw aligned using IMU and GPS");
                end

           end
        end
    else    % 如果gps数据不能用的情况
        todo2 = 1;
     
    end
    
end