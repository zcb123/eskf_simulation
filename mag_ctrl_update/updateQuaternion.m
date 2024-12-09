function ret = updateQuaternion(innovation,variance,gate_sigma,yaw_jacobian)

    % Calculate innovation variance and Kalman gains, taking advantage of the fact that only the first 4 elements in H are non zero
	% calculate the innovation variance
    global control_status params fault_status P;
    global mag_test_ratio;
	heading_innov_var = variance;

	for row = 1:3 
		tmp = 0.0;

		for col = 1:3
			tmp = tmp + P(row, col) * yaw_jacobian(col);
		end

		heading_innov_var = heading_innov_var + yaw_jacobian(row) * tmp;
	end

    heading_innov_var_inv = 0;

	% check if the innovation variance calculation is badly conditioned
	if (heading_innov_var >= variance) 
		% the innovation variance contribution from the state covariances is not negative, no fault
		fault_status.flags.bad_hdg = false;
		heading_innov_var_inv = 1.0 / heading_innov_var;

	else 
		% the innovation variance contribution from the state covariances is negative which means the covariance matrix is badly conditioned
		fault_status.flags.bad_hdg = true;

		% we reinitialise the covariance matrix and abort this fusion step
		initialiseCovariance();
		%ECL_ERR("mag yaw fusion numerical error - covariance reset");
		ret= false;
        return;
	end

	% calculate the Kalman gains
	% only calculate gains for states we are using
	Kfusion = zeros(23,1);

	for row = 1:15 
		for col = 1:3 
			Kfusion(row) = Kfusion(row) + P(row, col) * yaw_jacobian(col);
		end

		Kfusion(row) =Kfusion(row)*heading_innov_var_inv;
	end

% 	if (control_status.flags.wind) 
% 		for row = 21; row <= 22; row++) 
% 			for col = 0; col <= 2; col++) 
% 				Kfusion(row) += P(row, col) * yaw_jacobian(col);
% 			end
% 
% 			Kfusion(row) *= heading_innov_var_inv;
% 		end
% 	end

	% innovation test ratio
	yaw_test_ratio = sq(innovation) / (sq(gate_sigma) * heading_innov_var);

	% we are no longer using 3-axis fusion so set the reported test levels to zero
	mag_test_ratio = zeros(3,1);

	% set the magnetometer unhealthy if the test fails
	if (yaw_test_ratio > 1.0) 
		%_innov_check_fail_status.flags.reject_yaw = true;
        disp('reject yaw updateQuaternion');
		% if we are in air we don't want to fuse the measurement
		% we allow to use it when on the ground because the large innovation could be caused
		% by interference or a large initial gyro bias
		if (~control_status.flags.in_air && isTimedOut(time_last_in_air, 5e6)) 
			% constrain the innovation to the maximum set by the gate
			% we need to delay this forced fusion to avoid starting it
			% immediately after touchdown, when the drone is still armed
			gate_limit = sqrt((sq(gate_sigma) * heading_innov_var));
			heading_innov = saturation(innovation, -gate_limit, gate_limit);

			% also reset the yaw gyro variance to converge faster and avoid
			% being stuck on a previous bad estimate
			%resetZDeltaAngBiasCov();
            disp('resetZDeltaAngBiasCov()');
		else 
			ret = false;
            return;
		end

	else 
		%_innov_check_fail_status.flags.reject_yaw = false;
		heading_innov = innovation;
	end

	% apply covariance correction via P_new = (I -K*H)*P
	% first calculate expression for KHP
	% then calculate P - KHP
	
	% P_new = (I - K*H)*P = P - K*H*P = p + INNOV_var * K * K.T
	KHP = zeros(23,23);
	%float KH[3];

	for row = 1:23

		%KH[0] = Kfusion(row) * yaw_jacobian(0);
		%KH[1] = Kfusion(row) * yaw_jacobian(1);
		%KH[2] = Kfusion(row) * yaw_jacobian(2);

		for column = 1:23
			%float tmp = KH[0] * P(0, column);
			%tmp += KH[1] * P(1, column);
			%tmp += KH[2] * P(2, column);
			%tmp += KH[3] * P(3, column);
			KHP(row, column) = Kfusion(row) * Kfusion(column) * variance;
		end
	end

	healthy = checkAndFixCovarianceUpdate(KHP);

	fault_status.flags.bad_hdg = ~healthy;

	if (healthy) 
		% apply the covariance corrections
		P = P- KHP;

		%fixCovarianceErrors(true);

		% apply the state corrections
		fuse(Kfusion, heading_innov);

		ret = true;
        return ;
	end

    ret = false;

end




