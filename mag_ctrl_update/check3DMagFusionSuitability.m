function check3DMagFusionSuitability()
    global yaw_angle_observable  mag_bias_observable time_last_mov_3d_mag_suitable;
    global imu_sample_delayed;
    check3DMagFusionSuitability();
    checkMagBiasObservability();

    if yaw_angle_observable && mag_bias_observable
        
        time_last_mov_3d_mag_suitable = imu_sample_delayed.time_us;
    
    end

end