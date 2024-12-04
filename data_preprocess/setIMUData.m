function updated = setIMUData(imu_sample)
    
    global control_status;
    global imu_sample_delayed newest_high_rate_imu_sample dt_imu_avg time_last_imu;
    global imu_down_sampler imu_buffer imu_sample_delayed_prev;
    global min_obs_interval_us obs_buffer_length;
    global initialised

%     imu_buffer = ring_buffer(12);

    if ~initialised
        initialised = init(imu_sample.time_us);

    end
    dt = double(imu_sample.time_us - time_last_imu)/1e6;
    dt = saturation(dt,1e-4,0.02);
    
    time_last_imu = imu_sample.time_us;
    
    if(time_last_imu>0)
        dt_imu_avg = 0.8*dt_imu_avg+0.2*dt;
    end

    newest_high_rate_imu_sample = imu_sample;

    computeVibrationMetric(imu_sample);
	control_status.flags.vehicle_at_rest = checkIfVehicleAtRest(dt, imu_sample);
    updated = imu_down_sampler.Update(imu_sample);

    if updated
        imu_buffer.push(imu_down_sampler.getDownSampledImuAndTriggerReset());
        imu_sample_delayed_prev = imu_sample_delayed;
        imu_sample_delayed = imu_buffer.get_oldest();%这里要先把缓冲区填满，tail才会+1
        
        min_obs_interval_us =  (imu_sample.time_us - imu_sample_delayed.time_us) / uint64(obs_buffer_length - 1);

        %setDragData(imu_sample);
    end

end

