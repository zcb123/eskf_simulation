function setMagData(mag_sample)

    global params dt_ekf_avg;
    global mag_buffer obs_buffer_length;
    global time_last_mag min_obs_interval_us;
    global initialised;

    if ~initialised 
		return;
    end

    if mag_buffer.len < 1
        mag_buffer = ring_buffer(obs_buffer_length);
    end

    if (mag_sample.time_us - time_last_mag) > min_obs_interval_us
		time_last_mag = mag_sample.time_us;
		mag_sample_new.time_us = mag_sample.time_us;
		mag_sample_new.time_us = mag_sample_new.time_us - (params.mag_delay_ms * 1000);
		mag_sample_new.time_us = mag_sample_new.time_us - (dt_ekf_avg * 5e5); 
		mag_sample_new.mag = mag_sample.mag;

		mag_buffer.push(mag_sample_new);

    else 

		disp("mag data too fast %");

	end
    
end


