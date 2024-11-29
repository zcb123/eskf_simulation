function ret = isRecent(sensor_timestamp,acceptance_interval)

    global time_last_imu

    ret = sensor_timestamp + acceptance_interval>time_last_imu;


end


