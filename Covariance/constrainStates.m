function constrainStates()

    global states params dt_ekf_avg;

    states.quat_nominal = saturation(states.quat_nominal, -1.0, 1.0);
	states.vel = saturation(states.vel, -1000.0, 1000.0);
	states.pos = saturation(states.pos, -1.e6, 1.e6);

	delta_ang_bias_limit = radians(20) * dt_ekf_avg;
	states.delta_ang_bias = saturation(states.delta_ang_bias, -delta_ang_bias_limit, delta_ang_bias_limit);

	delta_vel_bias_limit = params.acc_bias_lim * dt_ekf_avg;
	states.delta_vel_bias = saturation(states.delta_vel_bias, -delta_vel_bias_limit, delta_vel_bias_limit);

	states.mag_I = saturation(states.mag_I, -1, 1);
	states.mag_B = saturation(states.mag_B, -0.5, 0.5);
	states.wind_vel = saturation(states.wind_vel, -100.0, 100.0);

end


