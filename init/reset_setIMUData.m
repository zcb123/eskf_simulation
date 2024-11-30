function reset_setIMUData(dt_imu_avg)

    global required_samples target_dt_s min_dt_s;
    global params;
    
    delta_ang_dt_avg = dt_imu_avg;
    target_dt_s = saturation(params.filter_update_interval_us,1000, 100000) * 1e-6;		%0703:4000 0831:8000
    required_samples = max(round(target_dt_s / delta_ang_dt_avg), 1);					%0.004
    target_dt_s = required_samples * delta_ang_dt_avg;
    min_dt_s = max(delta_ang_dt_avg * (required_samples - 1), delta_ang_dt_avg * 0.5);

end