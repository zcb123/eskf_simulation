function ret = fuseYaw321(yaw,yaw_variance,zero_innovation)


    global states R_to_earth params;
% assign intermediate state variables
	q1 = states.quat_nominal(1);
	q2 = states.quat_nominal(2);
	q3 = states.quat_nominal(3);
	q4 = states.quat_nominal(4);

	R_YAW = fmaxf(yaw_variance, 1.0e-4);
	measurement = wrap_pn_pi(yaw);

	% calculate 321 yaw observation matrix
	% choose A or B computational paths to avoid singularity in derivation at +-90 degrees yaw
	SA0 = -2*powf(q3, 2) - 2*powf(q4, 2) + 1;
	SA1 = 2*q4;
	SA2 = 2*q3;
	SA3 = SA1*q1 + SA2*q2;
	SA4 = (powf(SA0, 2) + powf(SA3, 2));
% 	SA4_inv;
	if(SA4 < 1e-6) 
		SA4_inv = 0;
	else 
		SA4_inv = (-SA1*q2 + SA2*q1) / SA4;
    end

	H_YAW = zeros(3,1);
	H_YAW(1) = SA0*SA4_inv;
	H_YAW(2) = SA3*SA4_inv;
	H_YAW(3) = 1;

	% calculate the yaw innovation and wrap to the interval between +-pi
% 	innovation;

	if (zero_innovation) 
		innovation = 0;
	else 
		innovation = wrap_pn_pi(atan2(R_to_earth(2, 1), R_to_earth(1, 1)) - measurement);   %atan2(R_to_earth(2, 1), R_to_earth(1, 1) 预测值
    end

	% define the innovation gate size
	innov_gate = max(params.heading_innov_gate, 1.0);

	% Update the quaternion states and covariance matrix
	ret = updateQuaternion(innovation, R_YAW, innov_gate, H_YAW);



end



