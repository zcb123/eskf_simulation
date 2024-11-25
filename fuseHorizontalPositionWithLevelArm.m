function ret = fuseHorizontalPositionWithLevelArm(pos_offset)
    % assign intermediate variables
    global states P;
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
	
	bool ret = true;
    k_vel_id = 3;
    k_pos_id = 6;

	for index=1:3
		switch(index) 
			case 1
			
				% Axis 0 equations
				% Sub Expressions
				HKX0 = q1*q2;
				HKX1 = q3*q4;
				HKX2 = 2*y_offset_body*(HKX0 + HKX1);
				HKX3 = 2*x_offset_body;
				HKX4 = HKX3*(q1*q3 - q2*q4);
				HKX5 = 2*powf(q2, 2) - 1;
				HKX6 = z_offset_body*(HKX5 + 2*powf(q3, 2));
				HKX7 = HKX3*(q1*q4 + q2*q3);
				HKX8 = 2*z_offset_body*(HKX0 - HKX1);
				HKX9 = y_offset_body*(HKX5 + 2*powf(q4, 2));
				HKX10 = -HKX2 + HKX4 + HKX6;
				HKX11 = HKX7 - HKX8 - HKX9;
				HKX12 = -HKX10*P(1,1) - HKX11*P(1,2) + P(1,7);
				HKX13 = -HKX10*P(1,2) - HKX11*P(2,2) + P(2,7);
				HKX14 = -HKX10*P(1,7) - HKX11*P(2,7) + P(7,7);
				HKX15 = (-HKX10*HKX12 - HKX11*HKX13 + HKX14 + obs_var(1));

				innov_var(1) = HKX15;
				if(innov_var(1) < obs_var(1)) 
					% the innovation variance contribution from the state covariances is negative which means the covariance matrix is badly conditioned
					fault_status.flags.bad_pos_N = true;

					% we need to re-initialise covariances and abort this fusion step
					% velocity
					P(k_pos_id,k_pos_id) = sq(max(params.gps_pos_noise, 0.01));
					P(k_pos_id+1,k_pos_id+1) = P(k_vel_id,k_vel_id);
					%P(k_vel_id+2,k_vel_id+2) = sq(1.5f) * P(k_vel_id,k_vel_id);
					ret = false;
					return ;
				end
				fault_status.flags.bad_pos_N = false;

				test_ratio(1) = sq(innov(1)) / (sq(max(innov_gate, 1.0)) * innov_var(1));
				innov_check_fail = (test_ratio(1) > 1.0);
				innov_check_fail_status.flags.reject_hor_pos = innov_check_fail;
				if (~innov_check_fail || inhibit_gate) 
					if (inhibit_gate && test_ratio(1) > sq(100.0/ innov_gate)) 
						% always protect against extreme values that could result in a NaN
						ret = false;
                        return 
					end
                else 
                    ret = false;
					return ;
				end

				HKX15_inv = 1.0 / HKX15;
				% Observation Jacobians
				
				Hfusion(1) = HKX2 - HKX4 - HKX6;
				Hfusion(2) = -HKX7 + HKX8 + HKX9;
				Hfusion(7) = 1;


				% Kalman gains
				Kfusion(0) = HKX15_inv*(-HKX10*P(0,1) - HKX11*P(0,2) + P(0,7));
				Kfusion(1) = HKX12*HKX15_inv;
				Kfusion(2) = HKX13*HKX15_inv;
				Kfusion(4) = HKX15_inv*(-HKX10*P(1,4) - HKX11*P(2,4) + P(4,7));
				Kfusion(5) = HKX15_inv*(-HKX10*P(1,5) - HKX11*P(2,5) + P(5,7));
				Kfusion(6) = HKX15_inv*(-HKX10*P(1,6) - HKX11*P(2,6) + P(6,7));
				Kfusion(7) = HKX14*HKX15_inv;
				Kfusion(8) = HKX15_inv*(-HKX10*P(1,8) - HKX11*P(2,8) + P(7,8));
				Kfusion(9) = HKX15_inv*(-HKX10*P(1,9) - HKX11*P(2,9) + P(7,9));
				Kfusion(10) = HKX15_inv*(-HKX10*P(1,10) - HKX11*P(2,10) + P(7,10));
				Kfusion(11) = HKX15_inv*(-HKX10*P(1,11) - HKX11*P(2,11) + P(7,11));
				Kfusion(12) = HKX15_inv*(-HKX10*P(1,12) - HKX11*P(2,12) + P(7,12));
				Kfusion(13) = HKX15_inv*(-HKX10*P(1,13) - HKX11*P(2,13) + P(7,13));
				Kfusion(14) = HKX15_inv*(-HKX10*P(1,14) - HKX11*P(2,14) + P(7,14));
				Kfusion(15) = HKX15_inv*(-HKX10*P(1,15) - HKX11*P(2,15) + P(7,15));
				Kfusion(16) = HKX15_inv*(-HKX10*P(1,16) - HKX11*P(2,16) + P(7,16));
				Kfusion(17) = HKX15_inv*(-HKX10*P(1,17) - HKX11*P(2,17) + P(7,17));
				Kfusion(18) = HKX15_inv*(-HKX10*P(1,18) - HKX11*P(2,18) + P(7,18));
				Kfusion(19) = HKX15_inv*(-HKX10*P(1,19) - HKX11*P(2,19) + P(7,19));
				Kfusion(20) = HKX15_inv*(-HKX10*P(1,20) - HKX11*P(2,20) + P(7,20));
				Kfusion(21) = HKX15_inv*(-HKX10*P(1,21) - HKX11*P(2,21) + P(7,21));
				Kfusion(22) = HKX15_inv*(-HKX10*P(1,22) - HKX11*P(2,22) + P(7,22));
				Kfusion(23) = HKX15_inv*(-HKX10*P(1,23) - HKX11*P(2,23) + P(7,23));
			

			case 2
			
				% Axis 1 equations
				% Sub Expressions
				HKY0 = 2*y_offset_body;
				HKY1 = HKY0*(q1*q2 + q3*q4);
				HKY2 = q1*q3;
				HKY3 = q2*q4;
				HKY4 = 2*x_offset_body*(HKY2 - HKY3);
				HKY5 = 2*q3*q3 - 1;
				HKY6 = z_offset_body*(HKY5 + 2*q2*q2);
				HKY7 = 2*z_offset_body*(HKY2 + HKY3);
				HKY8 = HKY0*(q1*q4 - q2*q3);
				HKY9 = x_offset_body*(HKY5 + 2*q4*q4);
				HKY10 = -HKY7 + HKY8 + HKY9;
				HKY11 = HKY1 - HKY4 - HKY6;
				HKY12 = -HKY10*P(0,2) - HKY11*P(0,0) + P(0,8);
				HKY13 = -HKY10*P(2,2) - HKY11*P(0,2) + P(2,8);
				HKY14 = -HKY10*P(2,8) - HKY11*P(0,8) + P(8,8);
				HKY15 = (-HKY10*HKY13 - HKY11*HKY12 + HKY14 + obs_var(2));

				innov_var(2) = HKY15;
				if(innov_var(2) < obs_var(2)) 
					% the innovation variance contribution from the state covariances is negative which means the covariance matrix is badly conditioned
					fault_status.flags.bad_pos_E = true;

					% we need to re-initialise covariances and abort this fusion step
					% velocity
                    
					P(k_pos_id,k_pos_id) = sq(max(params.gps_pos_noise, 0.01));
					P(k_pos_id+1,k_pos_id+1) = P(k_vel_id,k_vel_id);
					%P(k_vel_id+2,k_vel_id+2) = sq(1.5f) * P(k_vel_id,k_vel_id);
					ret = false;
					return ;
				end
				fault_status.flags.bad_pos_E = false;

				test_ratio(2) = sq(innov(2)) / (sq(max(innov_gate, 2.0)) * innov_var(2));
				innov_check_fail = (test_ratio(2) > 2.0);
				innov_check_fail_status.flags.reject_hor_pos = innov_check_fail;
				if (~innov_check_fail || inhibit_gate) 
					if (inhibit_gate && test_ratio(2) > sq(200 / innov_gate)) 
						% always protect against extreme values that could result in a NaN
						ret = false;
                        return ;
					end
                else 
                    ret = false;
					return ;
				end

				HKY15_inv = 1.0 / HKY15;
				% Observation Jacobians
			
				Hfusion(0) = -HKY1 + HKY4 + HKY6;
				Hfusion(2) = HKY7 - HKY8 - HKY9;
				Hfusion(8) = 1;

				% Kalman gains
				Kfusion(0) = HKY12*HKY15_inv;
				Kfusion(1) = HKY15_inv*(-HKY10*P(1,2) - HKY11*P(0,1) + P(1,8));
				Kfusion(2) = HKY13*HKY15_inv;
				Kfusion(4) = HKY15_inv*(-HKY10*P(2,4) - HKY11*P(0,4) + P(4,8));
				Kfusion(5) = HKY15_inv*(-HKY10*P(2,5) - HKY11*P(0,5) + P(5,8));
				Kfusion(6) = HKY15_inv*(-HKY10*P(2,6) - HKY11*P(0,6) + P(6,8));
				Kfusion(7) = HKY15_inv*(-HKY10*P(2,7) - HKY11*P(0,7) + P(7,8));
				Kfusion(8) = HKY14*HKY15_inv;
				Kfusion(9) = HKY15_inv*(-HKY10*P(2,9) - HKY11*P(0,9) + P(8,9));
				Kfusion(10) = HKY15_inv*(-HKY10*P(2,10) - HKY11*P(0,10) + P(8,10));
				Kfusion(11) = HKY15_inv*(-HKY10*P(2,11) - HKY11*P(0,11) + P(8,11));
				Kfusion(12) = HKY15_inv*(-HKY10*P(2,12) - HKY11*P(0,12) + P(8,12));
				Kfusion(13) = HKY15_inv*(-HKY10*P(2,13) - HKY11*P(0,13) + P(8,13));
				Kfusion(14) = HKY15_inv*(-HKY10*P(2,14) - HKY11*P(0,14) + P(8,14));
				Kfusion(15) = HKY15_inv*(-HKY10*P(2,15) - HKY11*P(0,15) + P(8,15));
				Kfusion(16) = HKY15_inv*(-HKY10*P(2,16) - HKY11*P(0,16) + P(8,16));
				Kfusion(17) = HKY15_inv*(-HKY10*P(2,17) - HKY11*P(0,17) + P(8,17));
				Kfusion(18) = HKY15_inv*(-HKY10*P(2,18) - HKY11*P(0,18) + P(8,18));
				Kfusion(19) = HKY15_inv*(-HKY10*P(2,19) - HKY11*P(0,19) + P(8,19));
				Kfusion(20) = HKY15_inv*(-HKY10*P(2,20) - HKY11*P(0,20) + P(8,20));
				Kfusion(21) = HKY15_inv*(-HKY10*P(2,21) - HKY11*P(0,21) + P(8,21));
				Kfusion(22) = HKY15_inv*(-HKY10*P(2,22) - HKY11*P(0,22) + P(8,22));
				Kfusion(23) = HKY15_inv*(-HKY10*P(2,23) - HKY11*P(0,23) + P(8,23));	
		end

	
		ret = measurementUpdate(Kfusion, innov_var(index), innov(index), deltastates);


end