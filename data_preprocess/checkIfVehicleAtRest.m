function ret = checkIfVehicleAtRest(dt,imu)
    global control_status
    global time_last_move_detect_us vibe_metrics;
% detect if the vehicle is not moving when on ground
% 加速度和陀螺仪的变化量大于设定的阈值则重置检测时间，长时间未检测到时间变化则认为飞行器静止
	if ~control_status.flags.in_air             %在滤波器初始化成功后in_air这个值会置false
		if((vibe_metrics(2) * 4.0e4 > 1.0) ...
		|| (vibe_metrics(3) * 2.1e2 > 1.0) ...
		|| ((norm(imu.delta_ang) / dt) > 0.05 * 1.0)) 
			time_last_move_detect_us = imu.time_us;
		end

		ret = ((imu.time_us - time_last_move_detect_us) > 1e6);

	else 
		time_last_move_detect_us = imu.time_us;
		ret = false;
	end

end