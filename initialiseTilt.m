function ret = initialiseTilt(accel_lpf,gyro_lpf)

    global states R_to_earth;
    global CONSTANTS_ONE_G;

    accel_norm = norm(accel_lpf);
	gyro_norm = norm(gyro_lpf);

	if (accel_norm < 0.8 * CONSTANTS_ONE_G ||...
	    accel_norm > 1.2 * CONSTANTS_ONE_G ||...
	    gyro_norm > 15/57.3) 
		ret =  false;
        return ;
    end

	% get initial roll and pitch estimate from delta velocity vector, assuming vehicle is static
	gravity_in_body = accel_lpf./accel_norm;    

	pitch = asin(gravity_in_body(1));
	roll = atan2(-gravity_in_body(2), -gravity_in_body(3));

	states.quat_nominal = Euler_to_Quaternion([roll pitch 0]);

	R_to_earth = Quat2Tbn(states.quat_nominal);

    ret = true;
end

