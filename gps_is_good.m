function ret = gps_is_good(gps)


    global last_gps_fail_us last_gps_pass_us time_last_imu;

    fix_failed = gps.fix_type < 3;

    if last_gps_fail_us == 0
        last_gps_fail_us = time_last_imu;
    end

    if fix_failed
        last_gps_fail_us = time_last_imu;
    else
        last_gps_pass_us = time_last_imu;
    end
    

    ret = isTimedOut(last_gps_fail_us,10000000);
    
end


