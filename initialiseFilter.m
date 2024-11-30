function ret = initialiseFilter()

    global states;
    global imu_sample_delayed;
    global is_first_imu_sample;
    global time_last_imu time_last_hgt_fuse time_last_hor_pos_fuse time_last_hor_vel_fuse time_last_hagl_fuse time_last_flow_terrain_fuse time_last_of_fuse;
    
    imu_init = imu_sample_delayed;


    if is_first_imu_sample
        accel_lpf = imu_init.delta_vel/imu_init.delta_vel_dt;
        gyr_lpf = imu_init.delta_ang/imu_init.delta_ang_dt;
        is_first_imu_sample = false;
    else
        accel_lpf = alphaFilter(imu_init.delta_vel/imu_init.delta_vel_dt,0.1);
        gyr_lpf = Copy_of_alphaFilter(imu_init.delta_ang/imu_init.delta_ang_dt);
    end
    
    if ~initialiseTilt(accel_lpf,gyr_lpf)
        disp("Initial tilt false\n")
        ret = false;
        return;
    end
    %setControlBaroHeight();

    init_P();
    
    %resetMagHeading();
    
    time_last_hgt_fuse = time_last_imu;
	time_last_hor_pos_fuse = time_last_imu;
	time_last_hor_vel_fuse = time_last_imu;
	time_last_hagl_fuse = time_last_imu;
	time_last_flow_terrain_fuse = time_last_imu;
	time_last_of_fuse = time_last_imu;

     % alignOutputFilter()
    global output_new output_buffer head_index tail_index;
    output_delayed = output_buffer(tail_index,:);

    q_delta =  QuatMult(states.quat_nominal,Quaternion_Inverse(output_delayed.quat_nominal));
    q_delta = quat_normalize(q_delta);

    vel_delta = states.vel - output_delayed.vel;
    pos_delta = states.pos - output_delayed.pos;

    for i = 1:3 
        output_buffer(i,:).quat_nominal = q_delta * output_buffer(i,:).quat_nominal;
		output_buffer(i,:).quat_nominal = quat_normalize(output_buffer(i,:).quat_nominal);
		output_buffer(i,:).vel = output_buffer(i,:).vel + vel_delta;
		output_buffer(i,:).pos = output_buffer(i,:).pos + pos_delta;
    end

end

