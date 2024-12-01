function ret = resetYawToEKFGSF()

    
    global yawEstimator control_status;
    global ekfgsf_yaw_reset_time time_last_imu ekfgsf_yaw_reset_count
    global flt_mag_align_start_time;

    if ~yawEstimator.ekf_gsf_vel_fuse_started
        ret = false;
        return
    end
    
    if ~yawEstimator.gsf_yaw_variance < (params.EKFGSF_yaw_err_max^2)
        ret = false;
        return
    end

    resetQuatStateYaw(yawEstimator.gsf_yaw,yawEstimator.gsf_yaw_variance,true);

    flt_mag_align_start_time = imu_sample_delayed.time_us; 
    control_status.flags.yaw_align = true;
   
    if control_status.flags.mag_hdg || control_status.flags.mag_3D
        control_status.flags.mag_fault = true;
    elseif control_status.flags.gps_yaw
        control_status.flags.gps_yaw_fault = true;
    end
    

    ekfgsf_yaw_reset_time = time_last_imu;
    ekfgsf_yaw_reset_count = ekfgsf_yaw_reset_count + 1;
    ret = true;

end


