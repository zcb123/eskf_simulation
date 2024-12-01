function res = compensateBaroForDynamicPressure(baro_alt_uncompensated)

    % calculate static pressure error = Pmeas - Ptruth
	% model position error sensitivity as a body fixed ellipse with a different scale in the positive and
	% negative X and Y directions. Used to correct baro data for positional errors
    global params CONSTANTS_ONE_G;
    global output_new states vel_imu_rel_body_ned;
	%R_to_body(output_new.quat_nominal.inversed());
    
    R_to_body = Quat2Tbn(output_new.quat_nominal)';

	% Calculate airspeed in body frame
	velocity_earth = output_new.vel - vel_imu_rel_body_ned;

	wind_velocity_earth= [states.wind_vel(1) states.wind_vel(2) 0]';

	airspeed_earth = velocity_earth - wind_velocity_earth;

	airspeed_body = R_to_body * airspeed_earth;

    if airspeed_body(1) >= 0
        K_pstatic_coef_1 = params.static_pressure_coef_xp;
    else
        K_pstatic_coef_1 = params.static_pressure_coef_xn;
    end
    if airspeed_body(2) >= 0
        K_pstatic_coef_2 = params.static_pressure_coef_yp;
    else
        K_pstatic_coef_2 = params.static_pressure_coef_yn;
    end

	K_pstatic_coef =[K_pstatic_coef_1 K_pstatic_coef_2 params.static_pressure_coef_z]';

	airspeed_squared = min(airspeed_body.*airspeed_body, sq(params.max_correction_airspeed));

    air_density = 1.225;

	pstatic_err = 0.5 * air_density * (sum(airspeed_squared.*(K_pstatic_coef)));

	% correct baro measurement using pressure error estimate and assuming sea level gravity
	res = baro_alt_uncompensated + pstatic_err / (air_density * CONSTANTS_ONE_G);




end


