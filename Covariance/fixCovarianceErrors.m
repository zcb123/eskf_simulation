function fixCovarianceErrors(force_symmetry)
    global params R_to_earth fault_status control_status states P CONSTANTS_ONE_G dt_ekf_avg;
    global accel_bias_inhibit time_acc_bias_check time_last_imu;
    global gps_vel_innov ev_vel_innov gps_pos_innov baro_hgt_innov rng_hgt_innov ev_pos_innov;
    
    P_lim = zeros(8,1); 
	P_lim(1) = 1.0;		% quaternion max var
	P_lim(2) = 1e6;		% velocity max var
	P_lim(3) = 1e6;		% positiion max var
	P_lim(4) = 1.0;		% gyro bias max var
	P_lim(5) = 1.0;		% delta velocity z bias max var
	P_lim(6) = 1.0;		% earth mag field max var
	P_lim(7) = 1.0;		% body mag field max var
	P_lim(8) = 1e6;		% wind max var

    k_ori_id = 0;
    k_vel_id = 3;
    k_pos_id = 6;
    k_ang_bias_id = 9;
    k_vel_bias_id = 12;
    k_mag_i_id = 15;
    k_mag_bias_id = 18;
    k_wind_vel_id = 21;
    
	for i = 1:3 
		% quaternion states
		P(k_ori_id + i, k_ori_id + i) = saturation(P(k_ori_id + i, k_ori_id + i), 0.0, P_lim(1));

		% NED velocity states
		P(k_vel_id + i, k_vel_id + i) = saturation(P(k_vel_id + i, k_vel_id + i), 1e-6, P_lim(2));

		% NED position states
		P(k_pos_id + i, k_pos_id + i) = saturation(P(k_pos_id + i, k_pos_id + i), 1e-6, P_lim(3));

		% gyro bias states
		P(k_ang_bias_id + i, k_ang_bias_id + i) = saturation(P(k_ang_bias_id + i, k_ang_bias_id + i), 0.0, P_lim(4));
	end

	% force symmetry on the quaternion, velocity and position state covariances
	if (force_symmetry) 
		%P.makeRowColSymmetric<k_vel_bias_id>(0);
        makeRowColSymmetric(12,1);
	end

	% the following states are optional and are deactivated when not required
	% by ensuring the corresponding covariance matrix values are kept at zero

	% accelerometer bias states
	if (~accel_bias_inhibit(1) || ~accel_bias_inhibit(2) || ~accel_bias_inhibit(3)) 
		% Find the maximum delta velocity bias state variance and request a covariance reset if any variance is below the safe minimum
		minSafeStateVar = 1e-9;
		maxStateVar = minSafeStateVar;
		resetRequired = false;

		for stateIndex = 1:3 
			if (accel_bias_inhibit(stateIndex)) 
				% Skip the check for the inhibited axis
				continue;
			end

			if (P(k_vel_bias_id + stateIndex, k_vel_bias_id + stateIndex) > maxStateVar) 
				maxStateVar = P(k_vel_bias_id + stateIndex, k_vel_bias_id + stateIndex);

            elseif (P(k_vel_bias_id + stateIndex, k_vel_bias_id + stateIndex) < minSafeStateVar) 
				resetRequired = true;
			end
		end

		%为确保协方差矩阵运算的稳定性，最大和最小方差之比不得超过100，最小方差不得低于目标最小值。同时，将方差限制在相当于0.1g不确定性的最大值
		minStateVarTarget = 5e-8;
		minAllowedStateVar = max(0.01 * maxStateVar, minStateVarTarget);

		for stateIndex = 1:3 
			if accel_bias_inhibit(stateIndex)
				% Skip the check for the inhibited axis
				continue;
			end

			P(k_vel_bias_id + stateIndex, k_vel_bias_id + stateIndex) = saturation(P(k_vel_bias_id + stateIndex, k_vel_bias_id + stateIndex), minAllowedStateVar,...
						    sq(0.1 * CONSTANTS_ONE_G * dt_ekf_avg));
		end

% 		If any one axis has fallen below the safe minimum, all delta velocity covariance terms must be reset to zero
		if (resetRequired) 
			%P.uncorrelateCovariance<3>(k_vel_bias_id);
            uncorrelateCovariance(3,k_vel_bias_id+1); %将协方差置零
		end
        
% 		Run additional checks to see if the delta velocity bias has hit limits in a direction that is clearly wrong
% 		calculate accel bias term aligned with the gravity vector
		dVel_bias_lim = 0.9 * params.acc_bias_lim * dt_ekf_avg;
		down_dvel_bias = sum(states.delta_vel_bias.*R_to_earth(3,:)');

		% check that the vertical component of accel bias is consistent with both the vertical position and velocity innovation
		bad_acc_bias = (fabsf(down_dvel_bias) > dVel_bias_lim ...
				     && ((down_dvel_bias * gps_vel_innov(2) < 0 && control_status.flags.gps)...
					 || (down_dvel_bias * ev_vel_innov(2) < 0 && control_status.flags.ev_vel))...
				     && ((down_dvel_bias * gps_pos_innov(2) < 0 && control_status.flags.gps_hgt)...
					 || (down_dvel_bias * baro_hgt_innov < 0 && control_status.flags.baro_hgt)...
					 || (down_dvel_bias * rng_hgt_innov < 0 && control_status.flags.rng_hgt)...
					 || (down_dvel_bias * ev_pos_innov(2) < 0 && control_status.flags.ev_hgt)));

		% record the pass/fail
		if (~bad_acc_bias) 
			fault_status.flags.bad_acc_bias = false;
			time_acc_bias_check = time_last_imu;

		else 
			fault_status.flags.bad_acc_bias = true;
		end

		% if we have failed for 7 seconds continuously, reset the accel bias covariances to fix bad conditioning of
		% the covariance matrix but preserve the variances (diagonals) to allow bias learning to continue
		if (isTimedOut(time_acc_bias_check,7e6)) 

			%P.uncorrelateCovariance<3>(k_vel_bias_id);
            uncorrelateCovariance(3,k_vel_bias_id+1);
			time_acc_bias_check = time_last_imu;
			fault_status.flags.bad_acc_bias = false;
			%warning_events.flags.invalid_accel_bias_cov_reset = true;
			disp("invalid accel bias - covariance reset");
        elseif (force_symmetry) 
			% ensure the covariance values are symmetrical
% 			P.makeRowColSymmetric<3>(k_vel_bias_id);
            makeRowColSymmetric(3,k_vel_bias_id+1);
		end

	end
%     makeRowColSymmetric(3,k_vel_bias_id+1);
	% magnetic field states
	if (~control_status.flags.mag_3D) 
		zeroMagCov();
        
	else 
		% constrain variances
		for i = 1:3 
			P(k_mag_i_id + i, k_mag_i_id + i) = saturation(P(k_mag_i_id + i, k_mag_i_id + i), 0.0, P_lim(6));
			P(k_mag_bias_id + i, k_mag_bias_id + i) = saturation(P(k_mag_bias_id + i, k_mag_bias_id + i), 0.0, P_lim(7));
		end


		% force symmetry
		if (force_symmetry) 
% 			P.makeRowColSymmetric<3>(k_mag_i_id);
            makeRowColSymmetric(3,k_mag_i_id+1);
% 			P.makeRowColSymmetric<3>(k_mag_bias_id);
            makeRowColSymmetric(3,k_mag_bias_id+1);
		end

	end

	% wind velocity states
	if (~control_status.flags.wind) 
		%P.uncorrelateCovarianceSetVariance<2>(_k_wind_vel_id, 0.0f);
        P(k_wind_vel_id+1,k_wind_vel_id+1) = 0;
        P(k_wind_vel_id+2,k_wind_vel_id+2) = 0;
	else 
		% constrain variances
		for i = 1:2 
			P(k_wind_vel_id + i, k_wind_vel_id + i) = saturation(P(k_wind_vel_id + i, k_wind_vel_id + i), 0.0, P_lim(8));
		end

		% force symmetry
		if (force_symmetry) 
			%P.makeRowColSymmetric<2>(_k_wind_vel_id);
            makeRowColSymmetric(2,k_wind_vel_id+1);
		end
	end


end
