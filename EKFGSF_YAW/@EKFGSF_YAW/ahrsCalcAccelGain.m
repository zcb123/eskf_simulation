function res = ahrsCalcAccelGain(obj)
    global FLT_EPSILON CONSTANTS_ONE_G;
    attenuation = 2;
	centripetal_accel_compensation_enabled = (obj.true_airspeed > FLT_EPSILON);

	if  centripetal_accel_compensation_enabled...
	    && obj.ahrs_accel_norm > CONSTANTS_ONE_G 

		attenuation = 1;

    end

	delta_accel_g = (obj.ahrs_accel_norm - CONSTANTS_ONE_G) / CONSTANTS_ONE_G;  %当飞机静止时，ahrs_accel_norm接近于G，这里的值趋向于零

	res= obj.tilt_gain * sq(1 - min(attenuation*fabsf(delta_accel_g), 1));      %这里限制了最大值1

end

