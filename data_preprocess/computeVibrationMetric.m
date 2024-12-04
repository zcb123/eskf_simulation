function computeVibrationMetric(imu)
    % calculate a metric which indicates the amount of coning vibration
    global vibe_metrics delta_ang_prev delta_vel_prev;
	temp = cross(imu.delta_ang,delta_ang_prev);
	vibe_metrics(1) = 0.99 * vibe_metrics(1) + 0.01 * norm(temp);

	% calculate a metric which indicates the amount of high frequency gyro vibration
	temp = imu.delta_ang - delta_ang_prev;
	delta_ang_prev = imu.delta_ang;
	vibe_metrics(2) = 0.99 * vibe_metrics(2) + 0.01 * norm(temp);

	% calculate a metric which indicates the amount of high frequency accelerometer vibration
	temp = imu.delta_vel - delta_vel_prev;
	delta_vel_prev = imu.delta_vel;
	vibe_metrics(3) = 0.99 * vibe_metrics(3) + 0.01 * norm(temp);

end