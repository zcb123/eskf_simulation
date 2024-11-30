function controlMagFusion(mag_data_ready)

    global control_status mag_sample_delayed;
    global is_yaw_fusion_inhibited mag_yaw_reset_req

    if ~control_status.flags.in_air
        control_status.flags.mag_aligned_in_flight = false;
    
    end

    if(~control_status.flags.tilt_align) 
        
        if~control_status.flags.gps_yaw

            is_yaw_fusion_inhibited = true;
                
            fuseHeading()

            is_yaw_fusion_inhibited = false;
        end
    end
    

    mag_yaw_reset_req = mag_yaw_reset_req | otherHeadingSourcesHaveStopped();
	mag_yaw_reset_req = mag_yaw_reset_req | ~control_status.flags.yaw_align;
	mag_yaw_reset_req = mag_yaw_reset_req | mag_inhibit_yaw_reset_req;

    if mag_data_ready

        selectMagAuto();

        if control_status.flags.in_air
            runInAirYawReset(mag_sample_delayed.mag);
        else
            runOnGroundYawReset();
        end



        if (~control_status.flags.yaw_align) 
			    %Having the yaw aligned is mandatory to continue
			    return;
        end



        runMagAndMagDeclFusions(mag_sample_delayed.mag)

    end






end



