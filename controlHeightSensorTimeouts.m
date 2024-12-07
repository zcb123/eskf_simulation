function controlHeightSensorTimeouts()

    global params control_status fault_status;
    global time_good_vert_accel;
    global gps_checks_passed gps_hgt_accurate gps_intermittent gps_sample_delayed;
    global time_last_hgt_fuse baro_hgt_faulty baro_hgt_intermittent;

    checkVerticalAccelerationHealth();

    continuous_bad_accel_hgt = isTimedOut(time_good_vert_accel, params.bad_acc_reset_delay_us);%bad_acc_reset_delay_us = 500000

    hgt_fusion_timeout = isTimedOut(time_last_hgt_fuse, 5e6);

    if (hgt_fusion_timeout || continuous_bad_accel_hgt) 
    
    % 		const char *failing_height_source = nullptr;
    % 		const char *new_height_source = nullptr;
            failing_height_source = "";
            new_height_source = "";
		    if control_status.flags.baro_hgt
			    reset_to_gps = false;
    
			    % reset to GPS if adequate GPS data is available and the timeout cannot be blamed on IMU data
			    if (~gps_intermittent) 
				    reset_to_gps = (gps_checks_passed && ~fault_status.flags.bad_acc_vertical && gps_hgt_accurate) || baro_hgt_faulty || baro_hgt_intermittent;
			    end
    
			    if (reset_to_gps) 
				    % set height sensor health
				    baro_hgt_faulty = true;
    
				    startGpsHgtFusion();
    
				    failing_height_source = "baro";
				    new_height_source = "gps";
    
			    elseif (~baro_hgt_faulty && ~baro_hgt_intermittent) 
				    resetHeightToBaro();
    
				    failing_height_source = "baro";
				    new_height_source = "baro";
			    end
    
		    elseif (control_status.flags.gps_hgt) 
			    reset_to_baro = false;
    
			    % if baro data is available and GPS data is inaccurate and the timeout cannot be blamed on IMU data, reset height to baro
			    if (~baro_hgt_faulty && ~baro_hgt_intermittent) 
				    reset_to_baro = (~fault_status.flags.bad_acc_vertical && ~gps_hgt_accurate) || gps_intermittent;
			    end
    
			    if (reset_to_baro) 
				    startBaroHgtFusion();
    
				    failing_height_source = "gps";
				    new_height_source = "baro";
    
			    elseif (~gps_intermittent) 
				    resetHeightToGps();
    
				    failing_height_source = "gps";
				    new_height_source = "gps";
			    end
    
		    elseif (control_status.flags.rng_hgt) 
    
			    if (range_sensor_isHealthy()) 
				    resetHeightToRng();
    
				    failing_height_source = "rng";
				    new_height_source = "rng";
    
                elseif (~baro_hgt_faulty && ~baro_hgt_intermittent) 
				    startBaroHgtFusion();
    
				    failing_height_source = "rng";
				    new_height_source = "baro";
			    end
    
		    elseif (control_status.flags.ev_hgt) 
			    % check if vision data is available
			    ev_data_available = false;
    
			    if (ext_vision_buffer) 
				    const ExtVisionSample &ev_init = ext_vision_buffer->get_newest();
				    ev_data_available = isRecent(ev_init.time_us, 2 * EV_MAX_INTERVAL);
			    end
    
			    if (ev_data_available) 
				    resetHeightToEv();
    
				    failing_height_source = "ev";
				    new_height_source = "ev";
    
                elseif (range_sensor_isHealthy()) 
				    % Fallback to rangefinder data if available
				    startRngHgtFusion();
    
				    failing_height_source = "ev";
				    new_height_source = "rng";
    
                elseif (~baro_hgt_faulty && ~baro_hgt_intermittent) 
				    startBaroHgtFusion();
    
				    failing_height_source = "ev";
				    new_height_source = "baro";
			    end
		    end
    
		    if strlength(failing_height_source) < 1 && strlength(new_height_source) < 1
			    %_warning_events.flags.height_sensor_timeout = true;
			    disp("%s hgt timeout - reset to %s");
		    end
    
		    % Also reset the vertical velocity
		    if (control_status.flags.gps && ~gps_intermittent && gps_checks_passed) 
			    resetVerticalVelocityToGps(gps_sample_delayed);
    
		    else 
			    resetVerticalVelocityToZero();
                %disp('resetVerticalVelocityToZero')
		    end
    end



end