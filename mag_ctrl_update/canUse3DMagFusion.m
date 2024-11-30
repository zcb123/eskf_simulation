function ret = canUse3DMagFusion()

    global control_status imu_sample_delayed;
    global time_last_mov_3d_mag_suitable;
    
    ret = control_status.flags.mag_aligned_in_flight &&(imu_sample_delayed.time_us - time_last_mov_3d_mag_suitable < 2e6);


end

