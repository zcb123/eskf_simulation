function fuseMag()
    global states params control_status;
% assign intermediate variables
	q1 = states.quat_nominal(1);
	q2 = states.quat_nominal(2);
	q3 = states.quat_nominal(3);
	q4 = states.quat_nominal(4);

	magN = states.mag_I(1);
	magE = states.mag_I(2);
	magD = states.mag_I(3);

	% XYZ Measurement uncertainty. Need to consider timing errors for fast rotations
	R_MAG = sq(fmaxf(params.mag_noise, 1));

	% calculate intermediate variables used for X axis innovation variance, observation Jacobians and Kalman gains
	%char* numerical_error_covariance_reset_string = "numerical error - covariance reset";

	% rotate magnetometer earth field state into body frame
	Dcmf R_to_body = quatToInverseRotMat(states.quat_nominal);

	mag_I_rot = R_to_body * states.mag_I;

	% compute magnetometer innovations
	mag_innov = mag_I_rot + states.mag_B - mag;

	% do not use the synthesized measurement for the magnetomter Z component for 3D fusion
	if (control_status.flags.synthetic_mag_z) 
		mag_innov(3) = 0;
	end

	% we are no longer using heading fusion so set the reported test level to zero
	yaw_test_ratio = 0;


	% For the first few seconds after in-flight alignment we allow the magnetic field state estimates to stabilise
	% before they are used to constrain heading drift
	update_all_states = ((imu_sample_delayed.time_us - flt_mag_align_start_time) > 5e6);

	% Observation jacobian and Kalman gain vectors

	Hfusion;

	Kfusion;

	innov_check_fail = false;
	% update the states and covariance using sequential fusion of the magnetometer components
	for index = 1:3 

		% Calculate Kalman gains and observation jacobians
		if (index == 1) 
			% Calculate X axis observation jacobians
			HKX0 = q1*q4 + q2*q3;
			HKX1 = q1*q3;
			HKX2 = q2*q4;
			HKX3 = HKX1 - HKX2;
			HKX4 = HKX0*magD + HKX3*magE;
			HKX5 = 2*magN;
			HKX6 = HKX3*HKX5;
			HKX7 = 2*powf(q3, 2);
			HKX8 = 2*powf(q4, 2);
			HKX9 = HKX7 + HKX8 - 1;
			HKX10 = HKX9*magD;
			HKX11 = HKX0*HKX5 + HKX9*magE;
			HKX12 = -HKX7 - HKX8 + 1;
			HKX13 = -HKX1 + HKX2;
			HKX14 = 2*HKX0;
			HKX15 = 2*HKX3;
			HKX16 = 2*HKX4;
			HKX17 = HKX12*magD - HKX13*HKX5;
			HKX18 = HKX11*P(1,3) - HKX14*P(1,17) + HKX15*P(1,18) - HKX16*P(1,1) + HKX17*P(1,2) + HKX9*P(1,16) - P(1,19);
			HKX19 = HKX14*P(17,19);
			HKX20 = HKX15*P(18,19);
			HKX21 = HKX9*P(16,19);
			HKX22 = HKX16*P(1,19);
			HKX23 = HKX11*P(3,19);
			HKX24 = -HKX10 + HKX6;
			HKX25 = HKX11*P(3,17) - HKX14*P(17,17) + HKX15*P(17,18) - HKX16*P(1,17) + HKX17*P(2,17) + HKX9*P(16,17) - P(17,19);
			HKX26 = HKX11*P(3,18) - HKX14*P(17,18) + HKX15*P(18,18) - HKX16*P(1,18) + HKX17*P(2,18) + HKX9*P(16,18) - P(18,19);
			HKX27 = HKX11*P(3,16) - HKX14*P(16,17) + HKX15*P(16,18) - HKX16*P(1,16) + HKX17*P(2,16) + HKX9*P(16,16) - P(16,19);
			HKX28 = HKX11*P(3,3) - HKX14*P(3,17) + HKX15*P(3,18) - HKX16*P(1,3) + HKX17*P(2,3) + HKX9*P(3,16) - P(3,19);
			HKX29 = HKX11*P(2,3) - HKX14*P(2,17) + HKX15*P(2,18) - HKX16*P(1,2) + HKX17*P(2,2) + HKX9*P(2,16) - P(2,19);
			HKX30 = (HKX11*HKX28 - HKX14*HKX25 + HKX15*HKX26 - HKX16*HKX18 + HKX19 - HKX20 - HKX21 + HKX22 - HKX23 + HKX24*HKX29 - HKX24*P(2,19) + HKX27*HKX9 + R_MAG + P(19,19));

			mag_innov_var(1) = HKX30;%HKX10*HKX20 - HKX11*HKX18 + HKX12*HKX22 + HKX13*HKX16 - HKX14*HKX19 + HKX15*HKX21 + HKX17*HKX6 + HKX23 + R_MAG;
			if (mag_innov_var(1) < R_MAG) 
				% the innovation variance contribution from the state covariances is negative which means the covariance matrix is badly conditioned
				fault_status.flags.bad_mag_x = true;

				% we need to re-initialise covariances and abort this fusion step
				resetMagRelatedCovariances();
                disp("numerical error - magX covariance reset")
				%ECL_ERR("magX %s", numerical_error_covariance_reset_string);
				return;
			end
			fault_status.flags.bad_mag_x = false;
			mag_test_ratio(1) = sq(mag_innov(1)) / (sq(max(params.mag_innov_gate, 1)) * mag_innov_var(1));
			innov_check_fail = (mag_test_ratio(1) > 1);
			%_innov_check_fail_status.flags.reject_mag_x = innov_check_fail;
			if(innov_check_fail) 
				return;
			end

			Hfusion.setZero();
			Hfusion(1) = 2*HKX4;
			Hfusion(2) = HKX10 - HKX6;
			Hfusion(3) = -HKX11;
			Hfusion(16) = HKX12;
			Hfusion(17) = 2*HKX0;
			Hfusion(18) = 2*HKX13;
			Hfusion(19) = 1;

			HKX30_inv = 1/ HKX30;
			% Calculate X axis Kalman gains
			if (update_all_states) 
				Kfusion(1) = -HKX18*HKX30_inv;
				Kfusion(2) = -HKX29*HKX30_inv;
				Kfusion(3) = -HKX28*HKX30_inv;
				Kfusion(4) = -HKX30_inv*(HKX11*P(3,4) - HKX14*P(4,17) + HKX15*P(4,18) - HKX16*P(1,4) + HKX17*P(2,4) + HKX9*P(4,16) - P(4,19));
				Kfusion(5) = -HKX30_inv*(HKX11*P(3,5) - HKX14*P(5,17) + HKX15*P(5,18) - HKX16*P(1,5) + HKX17*P(2,5) + HKX9*P(5,16) - P(5,19));
				Kfusion(6) = -HKX30_inv*(HKX11*P(3,6) - HKX14*P(6,17) + HKX15*P(6,18) - HKX16*P(1,6) + HKX17*P(2,6) + HKX9*P(6,16) - P(6,19));
				Kfusion(7) = -HKX30_inv*(HKX11*P(3,7) - HKX14*P(7,17) + HKX15*P(7,18) - HKX16*P(1,7) + HKX17*P(2,7) + HKX9*P(7,16) - P(7,19));
				Kfusion(8) = -HKX30_inv*(HKX11*P(3,8) - HKX14*P(8,17) + HKX15*P(8,18) - HKX16*P(1,8) + HKX17*P(2,8) + HKX9*P(8,16) - P(8,19));
				Kfusion(9) = -HKX30_inv*(HKX11*P(3,9) - HKX14*P(9,17) + HKX15*P(9,18) - HKX16*P(1,9) + HKX17*P(2,9) + HKX9*P(9,16) - P(9,19));
				Kfusion(10) = -HKX30_inv*(HKX11*P(3,10) - HKX14*P(10,17) + HKX15*P(10,18) - HKX16*P(1,10) + HKX17*P(2,10) + HKX9*P(10,16) - P(10,19));
				Kfusion(11) = -HKX30_inv*(HKX11*P(3,11) - HKX14*P(11,17) + HKX15*P(11,18) - HKX16*P(1,11) + HKX17*P(2,11) + HKX9*P(11,16) - P(11,19));
				Kfusion(12) = -HKX30_inv*(HKX11*P(3,12) - HKX14*P(12,17) + HKX15*P(12,18) - HKX16*P(1,12) + HKX17*P(2,12) + HKX9*P(12,16) - P(12,19));
				Kfusion(13) = -HKX30_inv*(HKX11*P(3,13) - HKX14*P(13,17) + HKX15*P(13,18) - HKX16*P(1,13) + HKX17*P(2,13) + HKX9*P(13,16) - P(13,19));
				Kfusion(14) = -HKX30_inv*(HKX11*P(3,14) - HKX14*P(14,17) + HKX15*P(14,18) - HKX16*P(1,14) + HKX17*P(2,14) + HKX9*P(14,16) - P(14,19));
				Kfusion(15) = -HKX30_inv*(HKX11*P(3,15) - HKX14*P(15,17) + HKX15*P(15,18) - HKX16*P(1,15) + HKX17*P(2,15) + HKX9*P(15,16) - P(15,19));

				Kfusion(22) = -HKX30_inv*(HKX11*P(3,22) - HKX14*P(17,22) + HKX15*P(18,22) - HKX16*P(1,22) + HKX17*P(2,22) + HKX9*P(16,22) - P(19,22));
				Kfusion(23) = -HKX30_inv*(HKX11*P(3,23) - HKX14*P(17,23) + HKX15*P(18,23) - HKX16*P(1,23) + HKX17*P(2,23) + HKX9*P(16,23) - P(19,23));
			end

			Kfusion(16) = -HKX27*HKX30_inv;
			Kfusion(17) = -HKX25*HKX30_inv;
			Kfusion(18) = -HKX26*HKX30_inv;
			Kfusion(19) = -HKX30_inv*(HKX17*P(2,19) - HKX19 + HKX20 + HKX21 - HKX22 + HKX23 - P(19,19));
			Kfusion(20) = -HKX30_inv*(HKX11*P(3,20) - HKX14*P(17,20) + HKX15*P(18,20) - HKX16*P(1,20) + HKX17*P(2,20) + HKX9*P(16,20) - P(19,20));
			Kfusion(21) = -HKX30_inv*(HKX11*P(3,21) - HKX14*P(17,21) + HKX15*P(18,21) - HKX16*P(1,21) + HKX17*P(2,21) + HKX9*P(16,21) - P(19,21));

            elseif (index == 2) 
			HKY0 = q1*q2 + q3*q4;
			HKY1 = 2*magE;
			HKY2 = 2*powf(q2, 2);
			HKY3 = 2*powf(q4, 2);
			HKY4 = HKY2 + HKY3 - 1;
			HKY5 = HKY0*HKY1 + HKY4*magD;
			HKY6 = HKY0*magN;
			HKY7 = q1*q4;
			HKY8 = q2*q3;
			HKY9 = HKY7 - HKY8;
			HKY10 = HKY6 + HKY9*magD;
			HKY11 = HKY1*HKY9;
			HKY12 = HKY4*magN;
			HKY13 = -HKY11 + HKY12;
			HKY14 = -HKY7 + HKY8;
			HKY15 = 2*HKY0;
			HKY16 = 2*HKY9;
			HKY17 = 2*HKY14*magD - 2*HKY6;
			HKY18 = -HKY13*P(1,3) - HKY15*P(1,18) + HKY16*P(1,16) + HKY17*P(1,2) + HKY4*P(1,17) + HKY5*P(1,1) - P(1,20);
			HKY19 = HKY15*P(18,20);
			HKY20 = HKY16*P(16,20);
			HKY21 = HKY4*P(17,20);
			HKY22 = 2*HKY10;
			HKY23 = HKY5*P(1,20);
			HKY24 = HKY11 - HKY12;
			HKY25 = -HKY13*P(3,18) - HKY15*P(18,18) + HKY16*P(16,18) + HKY17*P(2,18) + HKY4*P(17,18) + HKY5*P(1,18) - P(18,20);
			HKY26 = -HKY13*P(3,16) - HKY15*P(16,18) + HKY16*P(16,16) + HKY17*P(2,16) + HKY4*P(16,17) + HKY5*P(1,16) - P(16,20);
			HKY27 = -HKY13*P(3,17) - HKY15*P(17,18) + HKY16*P(16,17) + HKY17*P(2,17) + HKY4*P(17,17) + HKY5*P(1,17) - P(17,20);
			HKY28 = -HKY13*P(2,3) - HKY15*P(2,18) + HKY16*P(2,16) + HKY17*P(2,2) + HKY4*P(2,17) + HKY5*P(1,2) - P(2,20);
			HKY29 = -HKY13*P(3,3) - HKY15*P(3,18) + HKY16*P(3,16) + HKY17*P(2,3) + HKY4*P(3,17) + HKY5*P(1,3) - P(3,20);
			HKY30 = (-HKY15*HKY25 + HKY16*HKY26 + HKY18*HKY5 + HKY19 - HKY20 - HKY21 - HKY22*HKY28 + HKY22*P(2,20) - HKY23 + HKY24*HKY29 - HKY24*P(3,20) + HKY27*HKY4 + R_MAG + P(20,20));
			mag_innov_var(2) = HKY30;
			% chedk innovation variances for being badly conditioned

			if (mag_innov_var(2) < R_MAG) 
				% the innovation variance contribution from the state covariances is negtive which means the covariance matrix is badly conditioned
				fault_status.flags.bad_mag_y = true;

				% we need to re-initialise covariances and abort this fusion step
				resetMagRelatedCovariances();
                disp("numerical error - magY covariance reset")
				%ECL_ERR("magY %s", numerical_error_covariance_reset_string);
				return;
			end
			fault_status.flags.bad_mag_y = false;

			mag_test_ratio(2) = sq(mag_innov(2)) / (sq(max(params.mag_innov_gate, 2)) * mag_innov_var(2));
			innov_check_fail = (mag_test_ratio(2) > 1);
			%_innov_check_fail_status.flags.reject_mag_y = innov_check_fail;
			if(innov_check_fail) 
				return;
			end

			Hfusion.setZero();
			Hfusion(1) = -HKY5;
			Hfusion(2) = 2*HKY10;
			Hfusion(3) = HKY13;
			Hfusion(16) = 2*HKY14;
			Hfusion(17) = -HKY2 - HKY3 + 1;
			Hfusion(18) = 2*HKY0;
			Hfusion(20) = 1;


			HKY30_inv = 1 / HKY30;
			% Calculate X axis Kalman gains
			if (update_all_states) 
				Kfusion(1) = -HKY18*HKY30_inv;
				Kfusion(2) = -HKY28*HKY30_inv;
				Kfusion(3) = -HKY29*HKY30_inv;
				Kfusion(4) = -HKY30_inv*(-HKY13*P(3,4) - HKY15*P(4,18) + HKY16*P(4,16) + HKY17*P(2,4) + HKY4*P(4,17) + HKY5*P(1,4) - P(4,20));
				Kfusion(5) = -HKY30_inv*(-HKY13*P(3,5) - HKY15*P(5,18) + HKY16*P(5,16) + HKY17*P(2,5) + HKY4*P(5,17) + HKY5*P(1,5) - P(5,20));
				Kfusion(6) = -HKY30_inv*(-HKY13*P(3,6) - HKY15*P(6,18) + HKY16*P(6,16) + HKY17*P(2,6) + HKY4*P(6,17) + HKY5*P(1,6) - P(6,20));
				Kfusion(7) = -HKY30_inv*(-HKY13*P(3,7) - HKY15*P(7,18) + HKY16*P(7,16) + HKY17*P(2,7) + HKY4*P(7,17) + HKY5*P(1,7) - P(7,20));
				Kfusion(8) = -HKY30_inv*(-HKY13*P(3,8) - HKY15*P(8,18) + HKY16*P(8,16) + HKY17*P(2,8) + HKY4*P(8,17) + HKY5*P(1,8) - P(8,20));
				Kfusion(9) = -HKY30_inv*(-HKY13*P(3,9) - HKY15*P(9,18) + HKY16*P(9,16) + HKY17*P(2,9) + HKY4*P(9,17) + HKY5*P(1,9) - P(9,20));
				Kfusion(10) = -HKY30_inv*(-HKY13*P(3,10) - HKY15*P(10,18) + HKY16*P(10,16) + HKY17*P(2,10) + HKY4*P(10,17) + HKY5*P(1,10) - P(10,20));
				Kfusion(11) = -HKY30_inv*(-HKY13*P(3,11) - HKY15*P(11,18) + HKY16*P(11,16) + HKY17*P(2,11) + HKY4*P(11,17) + HKY5*P(1,11) - P(11,20));
				Kfusion(12) = -HKY30_inv*(-HKY13*P(3,12) - HKY15*P(12,18) + HKY16*P(12,16) + HKY17*P(2,12) + HKY4*P(12,17) + HKY5*P(1,12) - P(12,20));
				Kfusion(13) = -HKY30_inv*(-HKY13*P(3,13) - HKY15*P(13,18) + HKY16*P(13,16) + HKY17*P(2,13) + HKY4*P(13,17) + HKY5*P(1,13) - P(13,20));
				Kfusion(14) = -HKY30_inv*(-HKY13*P(3,14) - HKY15*P(14,18) + HKY16*P(14,16) + HKY17*P(2,14) + HKY4*P(14,17) + HKY5*P(1,14) - P(14,20));
				Kfusion(15) = -HKY30_inv*(-HKY13*P(3,15) - HKY15*P(15,18) + HKY16*P(15,16) + HKY17*P(2,15) + HKY4*P(15,17) + HKY5*P(1,15) - P(15,20));

				Kfusion(22) = -HKY30_inv*(-HKY13*P(3,22) - HKY15*P(18,22) + HKY16*P(16,22) + HKY17*P(2,22) + HKY4*P(17,22) + HKY5*P(1,22) - P(20,22));
				Kfusion(23) = -HKY30_inv*(-HKY13*P(3,23) - HKY15*P(18,23) + HKY16*P(16,23) + HKY17*P(2,23) + HKY4*P(17,23) + HKY5*P(1,23) - P(20,23));

			end
			Kfusion(16) = -HKY26*HKY30_inv;
			Kfusion(17) = -HKY27*HKY30_inv;
			Kfusion(18) = -HKY25*HKY30_inv;
			Kfusion(19) = -HKY30_inv*(-HKY13*P(3,19) - HKY15*P(18,19) + HKY16*P(16,19) + HKY17*P(2,19) + HKY4*P(17,19) + HKY5*P(1,19) - P(19,20));
			Kfusion(20) = -HKY30_inv*(-HKY13*P(3,20) + HKY17*P(2,20) - HKY19 + HKY20 + HKY21 + HKY23 - P(20,20));
			Kfusion(21) = -HKY30_inv*(-HKY13*P(3,21) - HKY15*P(18,21) + HKY16*P(16,21) + HKY17*P(2,21) + HKY4*P(17,21) + HKY5*P(1,21) - P(20,21));

            elseif (index == 3) 

			% we do not fuse synthesized magnetomter measurements when doing 3D fusion
			if (control_status.flags.synthetic_mag_z) 
				continue;
			end

			HKZ0 = q1*q2;
			HKZ1 = q3*q4;
			HKZ2 = HKZ0 - HKZ1;
			HKZ3 = 2*magD;
			HKZ4 = HKZ2*HKZ3;
			HKZ5 = 2*powf(q2, 2);
			HKZ6 = 2*powf(q3, 2);
			HKZ7 = HKZ5 + HKZ6 - 1;
			HKZ8 = HKZ7*magE;
			HKZ9 = q1*q3 + q2*q4;
			HKZ10 = HKZ3*HKZ9 + HKZ7*magN;
			HKZ11 = HKZ2*magN + HKZ9*magE;
			HKZ12 = 2*HKZ9;
			HKZ13 = 2*HKZ2;
			HKZ14 = 2*HKZ11;
			HKZ15 = HKZ4 - HKZ8;
			HKZ16 = -HKZ10*P(1,2) + HKZ12*P(1,16) - HKZ13*P(1,17) + HKZ14*P(1,3) - HKZ15*P(1,1) - HKZ7*P(1,18) + P(1,21);
			HKZ17 = -HKZ10*P(2,2) + HKZ12*P(2,16) - HKZ13*P(2,17) + HKZ14*P(2,3) - HKZ15*P(1,2) - HKZ7*P(2,18) + P(2,21);
			HKZ18 = -HKZ10*P(2,18) + HKZ12*P(16,18) - HKZ13*P(17,18) + HKZ14*P(3,18) - HKZ15*P(1,18) - HKZ7*P(18,18) + P(18,21);
			HKZ19 = -HKZ10*P(2,17) + HKZ12*P(16,17) - HKZ13*P(17,17) + HKZ14*P(3,17) - HKZ15*P(1,17) - HKZ7*P(17,18) + P(17,21);
			HKZ20 = -HKZ10*P(2,3) + HKZ12*P(3,16) - HKZ13*P(3,17) + HKZ14*P(3,3) - HKZ15*P(1,3) - HKZ7*P(3,18) + P(3,21);
			HKZ21 = -HKZ10*P(2,16) + HKZ12*P(16,16) - HKZ13*P(16,17) + HKZ14*P(3,16) - HKZ15*P(1,16) - HKZ7*P(16,18) + P(16,21);
			HKZ22 = -HKZ10*P(2,21) + HKZ12*P(16,21) - HKZ13*P(17,21) + HKZ14*P(3,21) - HKZ15*P(1,21) - HKZ7*P(18,21) + P(21,21);
			HKZ23 = (-HKZ10*HKZ17 + HKZ12*HKZ21 - HKZ13*HKZ19 + HKZ14*HKZ20 - HKZ15*HKZ16 - HKZ18*HKZ7 + HKZ22 + R_MAG);
			mag_innov_var(3) = HKZ23;

			if (mag_innov_var(3) < R_MAG) 
				% the innovation variance contribution from the state covariances is negative which means the covariance matrix is badly conditioned
				fault_status.flags.bad_mag_z = true;

				% we need to re-initialise covariances and abort this fusion step
				resetMagRelatedCovariances();
                disp("numerical error - magZ covariance reset");
				%ECL_ERR("magZ %s", numerical_error_covariance_reset_string);
				return;
			end
			fault_status.flags.bad_mag_z = false;
			mag_test_ratio(3) = sq(mag_innov(3)) / (sq(max(params.mag_innov_gate, 1)) * mag_innov_var(3));
			innov_check_fail = (mag_test_ratio(3) > 1);
			%_innov_check_fail_status.flags.reject_mag_z = innov_check_fail;
			if(innov_check_fail) 
				return;
			end

			Hfusion.setZero();
			Hfusion(1) = -HKZ4 + HKZ8;
			Hfusion(2) = -HKZ10;
			Hfusion(3) = 2*HKZ11;
			Hfusion(16) = 2*HKZ9;
			Hfusion(17) = -2*HKZ0 + 2*HKZ1;
			Hfusion(18) = -HKZ5 - HKZ6 + 1;
			Hfusion(21) = 1;



			HKZ23_inv = 1 / HKZ23;
			% Calculate Z axis Kalman gains
			if (update_all_states) 
				Kfusion(1) = HKZ16*HKZ23_inv;
				Kfusion(2) = HKZ17*HKZ23_inv;
				Kfusion(3) = HKZ20*HKZ23_inv;
				Kfusion(4) = HKZ23_inv*(-HKZ10*P(2,4) + HKZ12*P(4,16) - HKZ13*P(4,17) + HKZ14*P(3,4) - HKZ15*P(1,4) - HKZ7*P(4,18) + P(4,21));
				Kfusion(5) = HKZ23_inv*(-HKZ10*P(2,5) + HKZ12*P(5,16) - HKZ13*P(5,17) + HKZ14*P(3,5) - HKZ15*P(1,5) - HKZ7*P(5,18) + P(5,21));
				Kfusion(6) = HKZ23_inv*(-HKZ10*P(2,6) + HKZ12*P(6,16) - HKZ13*P(6,17) + HKZ14*P(3,6) - HKZ15*P(1,6) - HKZ7*P(6,18) + P(6,21));
				Kfusion(7) = HKZ23_inv*(-HKZ10*P(2,7) + HKZ12*P(7,16) - HKZ13*P(7,17) + HKZ14*P(3,7) - HKZ15*P(1,7) - HKZ7*P(7,18) + P(7,21));
				Kfusion(8) = HKZ23_inv*(-HKZ10*P(2,8) + HKZ12*P(8,16) - HKZ13*P(8,17) + HKZ14*P(3,8) - HKZ15*P(1,8) - HKZ7*P(8,18) + P(8,21));
				Kfusion(9) = HKZ23_inv*(-HKZ10*P(2,9) + HKZ12*P(9,16) - HKZ13*P(9,17) + HKZ14*P(3,9) - HKZ15*P(1,9) - HKZ7*P(9,18) + P(9,21));
				Kfusion(10) = HKZ23_inv*(-HKZ10*P(2,10) + HKZ12*P(10,16) - HKZ13*P(10,17) + HKZ14*P(3,10) - HKZ15*P(1,10) - HKZ7*P(10,18) + P(10,21));
				Kfusion(11) = HKZ23_inv*(-HKZ10*P(2,11) + HKZ12*P(11,16) - HKZ13*P(11,17) + HKZ14*P(3,11) - HKZ15*P(1,11) - HKZ7*P(11,18) + P(11,21));
				Kfusion(12) = HKZ23_inv*(-HKZ10*P(2,12) + HKZ12*P(12,16) - HKZ13*P(12,17) + HKZ14*P(3,12) - HKZ15*P(1,12) - HKZ7*P(12,18) + P(12,21));
				Kfusion(13) = HKZ23_inv*(-HKZ10*P(2,13) + HKZ12*P(13,16) - HKZ13*P(13,17) + HKZ14*P(3,13) - HKZ15*P(1,13) - HKZ7*P(13,18) + P(13,21));
				Kfusion(14) = HKZ23_inv*(-HKZ10*P(2,14) + HKZ12*P(14,16) - HKZ13*P(14,17) + HKZ14*P(3,14) - HKZ15*P(1,14) - HKZ7*P(14,18) + P(14,21));
				Kfusion(15) = HKZ23_inv*(-HKZ10*P(2,15) + HKZ12*P(15,16) - HKZ13*P(15,17) + HKZ14*P(3,15) - HKZ15*P(1,15) - HKZ7*P(15,18) + P(15,21));

				Kfusion(22) = HKZ23_inv*(-HKZ10*P(2,22) + HKZ12*P(16,22) - HKZ13*P(17,22) + HKZ14*P(3,22) - HKZ15*P(1,22) - HKZ7*P(18,22) + P(21,22));
				Kfusion(23) = HKZ23_inv*(-HKZ10*P(2,23) + HKZ12*P(16,23) - HKZ13*P(17,23) + HKZ14*P(3,23) - HKZ15*P(1,23) - HKZ7*P(18,23) + P(21,23));
			end
			Kfusion(16) = HKZ21*HKZ23_inv;
			Kfusion(17) = HKZ19*HKZ23_inv;
			Kfusion(18) = HKZ18*HKZ23_inv;
			Kfusion(19) = HKZ23_inv*(-HKZ10*P(2,19) + HKZ12*P(16,19) - HKZ13*P(17,19) + HKZ14*P(3,19) - HKZ15*P(1,19) - HKZ7*P(18,19) + P(19,21));
			Kfusion(20) = HKZ23_inv*(-HKZ10*P(2,20) + HKZ12*P(16,20) - HKZ13*P(17,20) + HKZ14*P(3,20) - HKZ15*P(1,20) - HKZ7*P(18,20) + P(20,21));
			Kfusion(21) = HKZ22*HKZ23_inv;
		end

		is_fused = measurementUpdate(Kfusion, mag_innov_var(index), mag_innov(index));


		switch (index) 
		case 1
			fault_status.flags.bad_mag_x = ~is_fused;
			

		case 2
			fault_status.flags.bad_mag_y = ~is_fused;
			

		case 3
			fault_status.flags.bad_mag_z = ~is_fused;
			
		end

		if (is_fused) 
			limitDeclination();
		end
	end

end