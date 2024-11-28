function obj = Update(obj,imu_sample,CONSTANTS_ONE_G)
    % copy to class variables
    global states  dt_ekf_avg;

    imu_gyro_bias = states.delta_ang_bias/dt_ekf_avg;           %这个参数后面要用

% 	_delta_ang = imu_sample_delayed.delta_ang;
	obj.delta_vel = imu_sample.delta_vel;
% 	_delta_ang_dt = imu_sample_delayed.delta_ang_dt;
	obj.delta_vel_dt = imu_sample.delta_vel_dt;
	obj.run_ekf_gsf = logical(true);
% 	_true_airspeed = airspeed;

	% to reduce effect of vibration, filter using an LPF whose time constant is 1/10 of the AHRS tilt correction time constant
    
   
	filter_coef = min(10.0*obj.delta_vel_dt*obj.tilt_gain, 1.0);
	accel = obj.delta_vel / max(obj.delta_vel_dt, 0.001);

	obj.ahrs_accel = obj.ahrs_accel * (1.0 - filter_coef) + accel*filter_coef;

	% Initialise states first time
	if (~obj.ahrs_ekf_gsf_tilt_aligned) 
		% check for excessive acceleration to reduce likelihood of large initial roll/pitch errors
		% due to vehicle movement
		accel_norm_sq = norm(accel)^2;
		upper_accel_limit = CONSTANTS_ONE_G * 1.1;
		lower_accel_limit = CONSTANTS_ONE_G * 0.9;
		ok_to_align = (accel_norm_sq > sq(lower_accel_limit)) && (accel_norm_sq < sq(upper_accel_limit));

		if (ok_to_align) 
			initialiseEKFGSF();
			ahrsAlignTilt();
			obj.ahrs_ekf_gsf_tilt_aligned = true;
		end

		return;
	end

	% calculate common values used by the AHRS complementary filter models
	obj.ahrs_accel_norm = norm(obj.ahrs_accel);

	% AHRS prediction cycle for each model - this always runs
%	obj.ahrs_accel_fusion_gain = ahrsCalcAccelGain();

	for model_index = 1:5 
% 		predictEKF(model_index);
	end

	% The 3-state EKF models only run when flying to avoid corrupted estimates due to operator handling and GPS interference
	% 三态EKF模型仅在飞行时运行，以避免因操作员操作和GPS干扰而导致的估计错误
	if obj.run_ekf_gsf && obj.vel_data_updated 
		if (~obj.ekf_gsf_vel_fuse_started) 
			initialiseEKFGSF();
			ahrsAlignYaw();

			% Initialise to gyro bias estimate from main filter because there could be a large
			% uncorrected rate gyro bias error about the gravity vector
			for model_index = 1:5 
				obj.ahrs_ekf_gsf(model_index,1).gyro_bias = imu_gyro_bias;
			end

			obj.ekf_gsf_vel_fuse_started = true;

    	else 
			bad_update = logicle(false);

			for model_index = 1:5 
				% subsequent measurements are fused as direct state observations
				if (~updateEKF(model_index)) 
					bad_update = true;
				end
			end

			if (~bad_update) 
				total_weight = 0.0;
				% calculate weighting for each model assuming a normal distribution
				min_weight = 1e-5;
				n_weight_clips = 0;

				for model_index = 1:5
					obj.model_weights(model_index) = gaussianDensity(model_index) * obj.model_weights(model_index);

					if (obj.model_weights(model_index) < min_weight) 
						n_weight_clips = n_weight_clips + 1;
						obj.model_weights(model_index) = min_weight;
					end

					total_weight = total_weight + obj.model_weights(model_index);
				end

				% normalise the weighting function
				if n_weight_clips < 5
					obj.model_weights = obj.model_weights / total_weight;

    			else 
					% all weights have collapsed due to excessive innovation variances so reset filters
					initialiseEKFGSF();
                end
            end
		end
		
    elseif (obj.ekf_gsf_vel_fuse_started && ~obj.run_ekf_gsf) 
		% wait to fly again
		obj.ekf_gsf_vel_fuse_started = false;
	end

	% Calculate a composite yaw vector as a weighted average of the states for each model.
	% To avoid issues with angle wrapping, the yaw state is converted to a vector with length
	% equal to the weighting value before it is summed.
	yaw_vector = zeros(1,2);

	for model_index = 1:5 
		yaw_vector(1) = yaw_vector(1) + obj.model_weights(model_index) * cos(obj.ekf_gsf(model_index,1).X(3));
		yaw_vector(2) = yaw_vector(2) + obj.model_weights(model_index) * sin(obj.ekf_gsf(model_index,1).X(3));
	end

	obj.gsf_yaw = atan2f(yaw_vector(2), yaw_vector(1));

	% calculate a composite variance for the yaw state from a weighted average of the variance for each model
	% models with larger innovations are weighted less
	obj.gsf_yaw_variance = 0.0;

	for model_index = 0:5 
		yaw_delta = wrap_pn_pi(obj.ekf_gsf(model_index,1).X(3) - obj.gsf_yaw);
		obj.gsf_yaw_variance = obj.gsf_yaw_variance + obj.model_weights(model_index) * (obj.ekf_gsf(model_index,1).P(3, 3) + yaw_delta * yaw_delta);
	end

	% prevent the same velocity data being used more than once
	obj.vel_data_updated = logical(false);


end