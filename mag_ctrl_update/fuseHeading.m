function fuseHeading(measured_hdg,obs_var)

    global control_status params R_to_earth P;
    global last_static_yaw is_yaw_fusion_inhibited;
    % observation variance
    if ~isnan(obs_var)
	    R_YAW =obs_var;
    else
        R_YAW = 0.01;
    end

	% update transformation matrix from body to world frame using the current state estimate
	predicted_hdg = getEulerYaw(R_to_earth);

	if (isnan(measured_hdg)) 
		measured_hdg = predicted_hdg;
	end

	% handle special case where yaw measurement is unavailable
	fuse_zero_innov = false;

	if (is_yaw_fusion_inhibited) 
		% The yaw measurement cannot be trusted but we need to fuse something to prevent a badly
		% conditioned covariance matrix developing over time.
		if (~control_status.flags.vehicle_at_rest) 
			% Vehicle is not at rest so fuse a zero innovation if necessary to prevent
			% unconstrained quaternion variance growth and record the predicted heading
			% to use as an observation when movement ceases.
			% TODO a better way of determining when this is necessary
			sumQuatVar = P(1, 1) + P(2, 2) + P(3, 3);

			if (sumQuatVar > params.quat_max_variance) 
				fuse_zero_innov = true;
				R_YAW = 0.25;
			end

			last_static_yaw = predicted_hdg;

		else 
			% Vehicle is at rest so use the last moving prediction as an observation
			% to prevent the heading from drifting and to enable yaw gyro bias learning
			% before takeoff.
			if (isnan(last_static_yaw)) 
				last_static_yaw = predicted_hdg;
			end

			measured_hdg = last_static_yaw;
		end

	else 
		last_static_yaw = predicted_hdg;
	end

    

    fuseYaw321(measured_hdg, R_YAW, fuse_zero_innov);


end



