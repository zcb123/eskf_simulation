function set_in_air_status(in_air)
    

    global time_last_on_ground_us  time_last_in_air time_last_imu;
    global control_status
    if (~in_air) 
			time_last_on_ground_us = time_last_imu;

	else 
			time_last_in_air = time_last_imu;
    end

    control_status.flags.in_air = in_air;

end


