function initialise_interface(timestamp)
    global imu_sample_delayed;

    imu_sample_delayed.time_us = timestamp;
	imu_sample_delayed.delta_vel_clipping[0] = false;
	imu_sample_delayed.delta_vel_clipping[1] = false;
	imu_sample_delayed.delta_vel_clipping[2] = false;
    
    
end