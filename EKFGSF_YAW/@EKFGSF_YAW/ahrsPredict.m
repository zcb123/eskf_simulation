function obj = ahrsPredict(obj)


% generate attitude solution using simple complementary filter for the selected model

	ang_rate = obj.delta_ang / max(obj.delta_ang_dt, 0.001) - obj.ahrs_ekf_gsf(model_index,1).gyro_bias;

	R_to_body = obj.ahrs_ekf_gsf(model_index,1).R';
	gravity_direction_bf = R_to_body.col(2);

	% Perform angular rate correction using accel data and reduce correction as accel magnitude moves away from 1 g (reduces drift when vehicle picked up and moved).
	% During fixed wing flight, compensate for centripetal acceleration assuming coordinated turns and X axis forward
	tilt_correction = zeros(3,1);

	if (obj.ahrs_accel_fusion_gain > 0) 

		accel = obj.ahrs_accel;

		if (obj.true_airspeed > FLT_EPSILON) 
			% Calculate body frame centripetal acceleration with assumption X axis is aligned with the airspeed vector
			% Use cross product of body rate and body frame airspeed vector
			centripetal_accel_bf = [0.0  obj.true_airspeed * ang_rate(2) - obj.true_airspeed * ang_rate(1)];

			% correct measured accel for centripetal acceleration
			accel = accel - centripetal_accel_bf;
		end

	    tilt_correction = cross(gravity_direction_bf , accel) * obj.ahrs_accel_fusion_gain / obj.ahrs_accel_norm;

	end

	% Gyro bias estimation
	gyro_bias_limit = 0.05;
	spinRate = length(ang_rate);

	if spinRate < 0.175 
		obj.ahrs_ekf_gsf(model_index,1).gyro_bias = obj.ahrs_ekf_gsf(model_index,1).gyro_bias - tilt_correction * (obj.gyro_bias_gain * obj.delta_ang_dt);
		obj.ahrs_ekf_gsf(model_index,1).gyro_bias = saturation(obj.ahrs_ekf_gsf(model_index,1).gyro_bias, -gyro_bias_limit,gyro_bias_limit);
	end

	% delta angle from previous to current frame
	delta_angle_corrected = obj.delta_ang + (tilt_correction - obj.ahrs_ekf_gsf(model_index,1).gyro_bias)*obj.delta_ang_dt;

	% Apply delta angle to rotation matrix
	obj.ahrs_ekf_gsf(model_index,1).R = ahrsPredictRotMat(obj.ahrs_ekf_gsf(model_index,1).R, delta_angle_corrected);

end