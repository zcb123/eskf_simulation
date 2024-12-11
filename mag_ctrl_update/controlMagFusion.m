function controlMagFusion()

    global params;
    global control_status mag_sample_delayed imu_sample_delayed;
    global is_yaw_fusion_inhibited mag_yaw_reset_req mag_inhibit_yaw_reset_req;
    global mag_counter mag_buffer; 
    global USE_GEO_DECL NONE;
    global mag_declination_gps;
    if ~mag_buffer.isEmpty()
        [mag_sample_delayed,mag_data_ready] = mag_buffer.pop_first_older_than(imu_sample_delayed.time_us);
        if mag_data_ready
            mag_sample = mag_lpf_update(mag_sample_delayed.mag,0.1);
            mag_counter = mag_counter + 1;
        end

        if params.synthesize_mag_z && bitand(params.mag_declination_source,USE_GEO_DECL)...
			    && (NED_origin_initialised || ~isnan(mag_declination_gps))
			    dcm = Euler2Dcm([0 -mag_inclination_gps mag_declination_gps]); %机体系到地理系的旋转矩阵
                mag_vec = [mag_strength_gps 0 0]';
				mag_earth_pred = dcm * mag_vec;
				mag_sample.mag(3) = calculate_synthetic_mag_z_measurement(mag_sample.mag, mag_earth_pred);
				control_status.flags.synthetic_mag_z = true;
		else 
				control_status.flags.synthetic_mag_z = false;
        end
    end

    if mag_data_ready      
        checkMagFieldStrength(mag_sample_delayed.mag);  %不检查磁场强度
    end
    if ~control_status.flags.in_air
        control_status.flags.mag_aligned_in_flight = false;  
    end

    if(params.mag_fusion_type >= NONE||control_status.flags.mag_fault||~control_status.flags.tilt_align) 

        stopMagFusion();
        
        if noOtherYawAidingThanMag()
            is_yaw_fusion_inhibited = true;                
            fuseHeading(nan,nan)%航向角0更新
            is_yaw_fusion_inhibited = false;
        end

        return;

    end
    

    mag_yaw_reset_req = mag_yaw_reset_req | otherHeadingSourcesHaveStopped();
	mag_yaw_reset_req = mag_yaw_reset_req | ~control_status.flags.yaw_align;
	mag_yaw_reset_req = mag_yaw_reset_req | mag_inhibit_yaw_reset_req;

    if noOtherYawAidingThanMag()&&mag_data_ready
     
        selectMagAuto();        %磁力计只用于修正航向的原因在于in_air置false -> mag_aligned_in_flight =false

        if control_status.flags.in_air
            checkHaglYawResetReq();
            runInAirYawReset();     %这里没有运行
        else
            runOnGroundYawReset();
        end

        if (~control_status.flags.yaw_align) 
			    %Having the yaw aligned is mandatory to continue
                disp('yaw_align not complete return')
			    return;
        end

        checkMagDeclRequired();
        checkMagInhibition();
        runMagAndMagDeclFusions(mag_sample_delayed.mag)

    end

end



