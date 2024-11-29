function ret = resetYawToEKFGSF()

    
    global yawEstimator control_status;
    global ekfgsf_yaw_reset_time time_last_imu
    if ~yawEstimator.ekf_gsf_vel_fuse_started
        ret = false;
        return
    end
    
    if ~yawEstimator.gsf_yaw_variance < (params.EKFGSF_yaw_err_max^2)
        ret = false;
        return
    end

    resetQuatStateYaw(yawEstimator.gsf_yaw,yawEstimator.gsf_yaw_variance,true);


    control_status.flags.yaw_align = true;
    ekfgsf_yaw_reset_time = time_last_imu;

    if control_status.flags.gps_yaw
        control_status.flags.gps_yaw_fault = true;
    end
    
    ret = true;

end


