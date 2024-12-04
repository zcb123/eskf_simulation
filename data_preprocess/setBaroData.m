function setBaroData(baro_sample)


    global params dt_ekf_avg;
    global baro_buffer obs_buffer_length;
    global time_last_baro min_obs_interval_us;
    global initialised;

    if ~initialised 
		return;
    end

    global baro_sample_new;
    
    if baro_buffer.len < 1
        baro_buffer = ring_buffer(obs_buffer_length);
    end

    if (baro_sample.time_us - time_last_baro) > min_obs_interval_us
        time_last_baro = baro_sample.time_us;
        baro_sample_new.time_us = baro_sample.time_us;
		baro_sample_new.time_us = baro_sample_new.time_us - (params.baro_delay_ms * 1000);
		baro_sample_new.time_us = baro_sample_new.time_us - (dt_ekf_avg * 5e5); 
        baro_sample_new.hgt = compensateBaroForDynamicPressure(baro_sample.hgt);
        baro_buffer.push(baro_sample_new);
    else
        disp('baro data invalid');
    end

end

