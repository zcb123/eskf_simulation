function obj = Update(obj,imu_sample_delayed,airspeed)
    % copy to class variables
    global states  dt_ekf_avg CONSTANTS_ONE_G;

    imu_gyro_bias = states.delta_ang_bias/dt_ekf_avg;           %这个参数后面要用

    obj.delta_ang = imu_sample_delayed.delta_ang;
	obj.delta_vel = imu_sample_delayed.delta_vel;
    obj.delta_ang_dt = imu_sample_delayed.delta_ang_dt;
	obj.delta_vel_dt = imu_sample_delayed.delta_vel_dt;
	obj.run_ekf_gsf = logical(true);
    obj.true_airspeed = airspeed;

	% to reduce effect of vibration, filter using an LPF whose time constant is 1/10 of the AHRS tilt correction time constant
    
   
	filter_coef = min(10.0*obj.delta_vel_dt*obj.tilt_gain, 1.0);        %tilt_gain = 0.2
	accel = obj.delta_vel / max(obj.delta_vel_dt, 0.001);

	obj.ahrs_accel = obj.ahrs_accel * (1.0 - filter_coef) + accel*filter_coef;      %ahrs加速度低通滤波

	% Initialise states first time
	if (~obj.ahrs_ekf_gsf_tilt_aligned)     %ahrs_ekf_gsf_tilt_aligned 这个变量只有这里用
		% check for excessive acceleration to reduce likelihood of large initial roll/pitch errors
		% due to vehicle movement
		accel_norm_sq = norm(accel)^2;
		upper_accel_limit = CONSTANTS_ONE_G * 1.1;
		lower_accel_limit = CONSTANTS_ONE_G * 0.9;
		ok_to_align = (accel_norm_sq > sq(lower_accel_limit)) && (accel_norm_sq < sq(upper_accel_limit));       %静止在地面上

		if (ok_to_align) 
			obj.initialiseEKFGSF();
			obj.ahrsAlignTilt();
			obj.ahrs_ekf_gsf_tilt_aligned = true;
		end

		return;
	end

	% calculate common values used by the AHRS complementary filter models
	obj.ahrs_accel_norm = norm(obj.ahrs_accel);

	% AHRS prediction cycle for each model - this always runs
%	obj.ahrs_accel_fusion_gain = ahrsCalcAccelGain();
    obj.ahrs_accel_fusion_gain = obj.ahrsCalcAccelGain();

	for model_index = 1:5 
		obj.predictEKF(model_index);
	end

	% The 3-state EKF models only run when flying to avoid corrupted estimates due to operator handling and GPS interference
	% 三态EKF模型仅在飞行时运行，以避免因操作员操作和GPS干扰而导致的估计错误
	if obj.run_ekf_gsf && obj.vel_data_updated %vel_data_updated 这个值也是这里使用，GPS每次更新数据，这里置true
		if (~obj.ekf_gsf_vel_fuse_started) 
			obj.initialiseEKFGSF();
			obj.ahrsAlignYaw();

			% Initialise to gyro bias estimate from main filter because there could be a large
			% uncorrected rate gyro bias error about the gravity vector
			for model_index = 1:5 
				obj.ahrs_ekf_gsf(model_index,1).gyro_bias = imu_gyro_bias;
			end

			obj.ekf_gsf_vel_fuse_started = true;

    	else 
			bad_update = false;

			for model_index = 1:5 
				% subsequent measurements are fused as direct state observations
                [obj,res] = obj.updateEKF(model_index);
                %res = true;
				if (~res) 
					bad_update = true;
				end
			end

			if (~bad_update)        %如果更新成功，则开始下面的权重计算
				total_weight = 0.0;
				% calculate weighting for each model assuming a normal distribution
				min_weight = 1e-5;
				n_weight_clips = 0;

				for model_index = 1:5
					obj.model_weights(model_index) = obj.gaussianDensity(model_index) * obj.model_weights(model_index);

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
					obj.initialiseEKFGSF();
                end
            end
		end
		
    elseif (obj.ekf_gsf_vel_fuse_started && ~obj.run_ekf_gsf)               % 没有运行ekf_gsf，这里置为false
		% wait to fly again
		obj.ekf_gsf_vel_fuse_started = false;
	end

	% Calculate a composite yaw vector as a weighted average of the states for each model.
	% To avoid issues with angle wrapping, the yaw state is converted to a vector with length
	% equal to the weighting value before it is summed.
    % 计算复合偏航矢量，作为每个模型状态的加权平均值
    % 为了避免角度环绕的问题，在求和之前，将偏航状态转换为长度等于加权值的向量
	yaw_vector = zeros(2,1);

	for model_index = 1:5 
		yaw_vector(1) = yaw_vector(1) + obj.model_weights(model_index) * cos(obj.ekf_gsf(model_index,1).X(3));
		yaw_vector(2) = yaw_vector(2) + obj.model_weights(model_index) * sin(obj.ekf_gsf(model_index,1).X(3));
	end

	obj.gsf_yaw = atan2(yaw_vector(2), yaw_vector(1));
    if obj.gsf_yaw*57.3 > 50
        debug = 1;
    end
	% calculate a composite variance for the yaw state from a weighted average of the variance for each model
	% models with larger innovations are weighted less
	obj.gsf_yaw_variance = 0.0;

	for model_index = 1:5 
		yaw_delta = wrap_pn_pi(obj.ekf_gsf(model_index,1).X(3) - obj.gsf_yaw);
		obj.gsf_yaw_variance = obj.gsf_yaw_variance + obj.model_weights(model_index) * (obj.ekf_gsf(model_index,1).P(3, 3) + yaw_delta * yaw_delta);
	end

	% prevent the same velocity data being used more than once
	obj.vel_data_updated = logical(false);


end