function controlGpsFusion()
   
    global params control_status;
    global gps_sample_delayed imu_sample_delayed;
%     NED_origin_initialised = true;
    global NED_origin_initialised last_gps_pass_us last_gps_fail_us gps_checks_passed;
    global ekfgsf_yaw_reset_count ;
    global gps_data_ready gps_acc_changed;
    global mag_yaw_reset_req;

    if  gps_data_ready   
        
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
                if continuing_conditions_passing || isOtherSourceOfHorizontalAidingThan(control_status.flags.gps)
                    %%%%
                    disp('fuse gps vel pos')
                    fuseGpsVelPos();
                    %%%%
                    if shouldResetGpsFusion() || gps_acc_changed
                        was_gps_signal_lost = isTimedOut(time_prev_gps_us, 1000000);
                        if isYawFailure() && control_status.flags.in_air...
                                && ~was_gps_signal_lost...
                                && ekfgsf_yaw_reset_count < params.EKFGSF_reset_count_limit...
                                && isTimedOut(ekfgsf_yaw_reset_time,5000000)
                            
                            if resetYawToEKFGSF()               %这里没运行
                                disp("GPS emergency yaw reset");
                            end


                        else

                            if (control_status.flags.fixed_wing && control_status.flags.in_air) 
								%if flying a fixed wing aircraft, do a complete reset that includes yaw
							    mag_yaw_reset_req = true;
                            end

                            
                            if(gps_acc_changed) 
								disp("GPS fix status changed - resetting");
							else 
								disp("GPS fusion timeout - resetting");
                            end

                            gps_acc_changed = false;
          
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
            if starting_conditions_passing  %完成倾斜对齐和航向对齐且NED初始位置初始化成功之后才会开始GPS校正.NED_origin_initialised至少在10s后才可能为true

                  startGpsFusion(gps_sample_delayed);

            elseif gps_checks_passing&&~control_status.flags.yaw_align && (params.mag_fusion_type == 5) %NONE = 5
                
                if resetYawToEKFGSF()           %这里没运行

                    resetVelocityToGps(gps_sample_delayed);
					resetHorizontalPositionToGps(gps_sample_delayed);
                    disp("Yaw aligned using IMU and GPS");
                end

           end
        end
    elseif control_status.flags.gps && (imu_sample_delayed.time_us - gps_sample_delayed.time_us > 10e6)    % 如果gps数据不能用的情况

        stopGpsFusion();            %切换高度源
        disp("GPS data stopped");

    elseif control_status.flags.gps && (imu_sample_delayed.time_us - gps_sample_delayed.time_us > 1e6) ...
            && isOtherSourceOfHorizontalAidingThan(control_status.flags.gps)    %查看有没有其他的水平位置源可以用，目前这里是没有。因此false
        stopGpsFusion();
        disp('use other pos data');
    end
    
end