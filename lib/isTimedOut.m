function ret = isTimedOut(last_sensor_timestamp,timeout_period)

    global time_last_imu
    ret = last_sensor_timestamp + timeout_period < time_last_imu;
    
end
