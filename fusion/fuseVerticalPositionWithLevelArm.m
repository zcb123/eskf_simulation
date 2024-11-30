function ret = fuseVerticalPositionWithLevelArm(pos_offset)
    global states P;
    % assign intermediate variables
	q1 = states.quat_nominal(1);
	q2 = states.quat_nominal(2);
	q3 = states.quat_nominal(3);
	q4 = states.quat_nominal(4);

	x_offset_body = pos_offset(1);
	y_offset_body = pos_offset(2);
	z_offset_body = pos_offset(3);

	
	innov_check_fail = false;
	% Observation jacobian and Kalman gain vectors

	Hfusion = zeros(23,1);

	Kfusion = zeros(23,1);
    k_pos_id = 6;
	% Axis 2 equations
	% Sub Expressions
	HKZ0 = q1*q4;
	HKZ1 = q2*q3;
	HKZ2 = 2*x_offset_body*(HKZ0 + HKZ1);
	HKZ3 = 2*z_offset_body;
	HKZ4 = HKZ3*(q1*q2 - q3*q4);
	HKZ5 = 2*q4*q4 - 1;
	HKZ6 = y_offset_body*(HKZ5 + 2*q2*q2);
	HKZ7 = HKZ3*(q1*q3 + q2*q4);
	HKZ8 = 2*y_offset_body*(HKZ0 - HKZ1);
	HKZ9 = x_offset_body*(HKZ5 + 2*q3*q3);
	HKZ10 = -HKZ2 + HKZ4 + HKZ6;
	HKZ11 = HKZ7 - HKZ8 - HKZ9;
	HKZ12 = -HKZ10*P(0,0) - HKZ11*P(0,1) + P(0,9);
	HKZ13 = -HKZ10*P(0,1) - HKZ11*P(1,1) + P(1,9);
	HKZ14 = -HKZ10*P(0,9) - HKZ11*P(1,9) + P(9,9);
	HKZ15 = (-HKZ10*HKZ12 - HKZ11*HKZ13 + HKZ14 + obs_var);

	innov_var = HKZ15;
	if(innov_var < obs_var) 
		% the innovation variance contribution from the state covariances is negative which means the covariance matrix is badly conditioned
		fault_status.flags.bad_pos_D = true;

		% we need to re-initialise covariances and abort this fusion step
		% velocity
		%P(k_pos_id,k_pos_id) = sq(fmaxf(params.gps_pos_noise, 0.01f));
		P(k_pos_id+2,k_pos_id+2) = 1.5 * sq(max(params.gps_pos_noise, 0.01));
		%P(_k_vel_id+2,_k_vel_id+2) = sq(1.5f) * P(_k_vel_id,_k_vel_id);
		
        ret = false;

		return ;
	end
	fault_status.flags.bad_pos_D = false;

	test_ratio = sq(innov) / (sq(max(innov_gate, 1)) * innov_var);
	%test_ratio = sq(innov) / (sq(math::max(innov_gate, 1.0f)) * innov_var);
	innov_check_fail = (test_ratio > 1);
	innov_check_fail_status.flags.reject_hor_pos = innov_check_fail;
	if (innov_check_fail) 
        ret = false;
		return ;
	end

	HKZ15_inv = 1.0 / HKZ15;
	% Observation Jacobians
	

	Hfusion(0) = HKZ2 - HKZ4 - HKZ6;
	Hfusion(1) = -HKZ7 + HKZ8 + HKZ9;
	Hfusion(9) = 1;


	% Kalman gains
	Kfusion(0) = HKZ12*HKZ15_inv;
	Kfusion(1) = HKZ13*HKZ15_inv;
	Kfusion(2) = HKZ15_inv*(-HKZ10*P(0,2) - HKZ11*P(1,2) + P(2,9));
	Kfusion(4) = HKZ15_inv*(-HKZ10*P(0,4) - HKZ11*P(1,4) + P(4,9));
	Kfusion(5) = HKZ15_inv*(-HKZ10*P(0,5) - HKZ11*P(1,5) + P(5,9));
	Kfusion(6) = HKZ15_inv*(-HKZ10*P(0,6) - HKZ11*P(1,6) + P(6,9));
	Kfusion(7) = HKZ15_inv*(-HKZ10*P(0,7) - HKZ11*P(1,7) + P(7,9));
	Kfusion(8) = HKZ15_inv*(-HKZ10*P(0,8) - HKZ11*P(1,8) + P(8,9));
	Kfusion(9) = HKZ14*HKZ15_inv;
	Kfusion(10) = HKZ15_inv*(-HKZ10*P(0,10) - HKZ11*P(1,10) + P(9,10));
	Kfusion(11) = HKZ15_inv*(-HKZ10*P(0,11) - HKZ11*P(1,11) + P(9,11));
	Kfusion(12) = HKZ15_inv*(-HKZ10*P(0,12) - HKZ11*P(1,12) + P(9,12));
	Kfusion(13) = HKZ15_inv*(-HKZ10*P(0,13) - HKZ11*P(1,13) + P(9,13));
	Kfusion(14) = HKZ15_inv*(-HKZ10*P(0,14) - HKZ11*P(1,14) + P(9,14));
	Kfusion(15) = HKZ15_inv*(-HKZ10*P(0,15) - HKZ11*P(1,15) + P(9,15));
	Kfusion(16) = HKZ15_inv*(-HKZ10*P(0,16) - HKZ11*P(1,16) + P(9,16));
	Kfusion(17) = HKZ15_inv*(-HKZ10*P(0,17) - HKZ11*P(1,17) + P(9,17));
	Kfusion(18) = HKZ15_inv*(-HKZ10*P(0,18) - HKZ11*P(1,18) + P(9,18));
	Kfusion(19) = HKZ15_inv*(-HKZ10*P(0,19) - HKZ11*P(1,19) + P(9,19));
	Kfusion(20) = HKZ15_inv*(-HKZ10*P(0,20) - HKZ11*P(1,20) + P(9,20));
	Kfusion(21) = HKZ15_inv*(-HKZ10*P(0,21) - HKZ11*P(1,21) + P(9,21));
	Kfusion(22) = HKZ15_inv*(-HKZ10*P(0,22) - HKZ11*P(1,22) + P(9,22));
	Kfusion(23) = HKZ15_inv*(-HKZ10*P(0,23) - HKZ11*P(1,23) + P(9,23));
   
	ret = measurementUpdate(Kfusion, innov_var, innov);

% 	fault_status.flags.bad_pos_D = !is_fused;
    global time_last_hgt_fuse time_last_imu;
    
	if(is_fused) 
		time_last_hgt_fuse = time_last_imu;
	end
% 	return is_fused;

end


