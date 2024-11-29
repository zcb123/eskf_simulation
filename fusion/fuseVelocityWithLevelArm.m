function [innov_var,test_ratio_out,ret] = fuseVelocityWithLevelArm(params,pos_offset,innov,innov_gate,obs_var)
    %innov实际上是速度差
    global states P imu_sample_delayed;
    % assign intermediate variables
	q1 = states.quat_nominal(1);
	q2 = states.quat_nominal(2);
	q3 = states.quat_nominal(3);
	q4 = states.quat_nominal(4);

	x_offset_body = pos_offset(1);
	y_offset_body = pos_offset(2);
	z_offset_body = pos_offset(3);

	dt = imu_sample_delayed.delta_ang_dt;
	gyro_unbias = (imu_sample_delayed.delta_ang - states.delta_ang_bias) / dt;

	% XYZ Measurement uncertainty. Need to consider timing errors for fast rotations
	%R_VEL = obs_var(1);
	%char* numerical_error_covariance_reset_string = "numerical error - covariance reset";

	innov_check_fail = logical(false);
	% Observation jacobian and Kalman gain vectors
	Kfusion = zeros(23,1);
% 	deltastates = zeros(23,1);
	ret = logical(true);

    k_vel_id = uint8(4);
    
    innov_var = single(zeros(3,1));
    persistent test_ratio;
    if isempty(test_ratio)
        test_ratio = single(zeros(3,1));
    end
    test_ratio_out = test_ratio;
	for(index = 1: 3)       %这里是观测更新的delta_v
		switch (index)
			case 1
				% Axis X
				HKX0 = q1*q2;
				HKX1 = q3*q4;
				HKX2 = HKX0 + HKX1;
				HKX3 = HKX2*x_offset_body;
				HKX4 = q2*q4;
				HKX5 = q1*q3;
				HKX6 = HKX4 - HKX5;
				HKX7 = HKX6*y_offset_body;
				HKX8 = 2*gyro_unbias(3);%angular_z;
				HKX9 = 2*z_offset_body;
				HKX10 = HKX2*HKX9;
				HKX11 = 2*q3*q3;
				HKX12 = -HKX11;
				HKX13 = 2*q2*q2;
				HKX14 = 1 - HKX13;
				HKX15 = HKX12 + HKX14;
				HKX16 = HKX15*y_offset_body;
				HKX17 = HKX11 - 1;
				HKX18 = q1*q4;
				HKX19 = q2*q3;
				HKX20 = HKX18 + HKX19;
				HKX21 = 2*y_offset_body;
				HKX22 = 2*q4*q4;
				HKX23 = -HKX22;
				HKX24 = HKX14 + HKX23;
				HKX25 = gyro_unbias(1)*(HKX21*(-HKX0 + HKX1) - HKX24*z_offset_body) + 2*gyro_unbias(2)*(HKX20*z_offset_body + x_offset_body*(HKX0 - HKX1)) + gyro_unbias(3)*(-HKX20*HKX21 + HKX24*x_offset_body);
				HKX26 = HKX4 + HKX5;
				HKX27 = HKX26*y_offset_body;
				HKX28 = HKX18 - HKX19;
				HKX29 = 1/dt;
				HKX30 = 2*HKX29;
				HKX31 = 2*x_offset_body;
				HKX32 = HKX17 + HKX22;
				HKX33 = HKX29*(HKX26*HKX31 + HKX32*z_offset_body);
				HKX34 = -HKX18 + HKX19;
				HKX35 = HKX30*(HKX27 - HKX34*z_offset_body);
				HKX36 = HKX29*(HKX31*HKX34 - y_offset_body*(HKX12 + HKX23 + 2));
				HKX37 = HKX8*(-HKX3 + HKX7) + gyro_unbias(1)*(HKX10 - HKX16) + gyro_unbias(2)*(HKX15*x_offset_body - HKX6*HKX9);
				HKX38 = HKX35*P(2,10);
				HKX39 = HKX33*P(2,11);
				HKX40 = HKX36*P(2,12);
				HKX41 = HKX25*P(2,3);
				HKX42 = HKX37*P(2,2);
				HKX43 = HKX35*P(3,10);
				HKX44 = HKX33*P(3,11);
				HKX45 = HKX36*P(3,12);
				HKX46 = HKX25*P(3,3);
				HKX47 = HKX37*P(2,3);
				HKX48 = HKX35*P(10,12);
				HKX49 = HKX33*P(11,12);
				HKX50 = HKX36*P(12,12);
				HKX51 = HKX25*P(3,12);
				HKX52 = HKX37*P(2,12);
				HKX53 = HKX35*P(10,11);
				HKX54 = HKX33*P(11,11);
				HKX55 = HKX36*P(11,12);
				HKX56 = HKX25*P(3,11);
				HKX57 = HKX37*P(2,11);
				HKX58 = HKX35*P(10,10);
				HKX59 = HKX33*P(10,11);
				HKX60 = HKX36*P(10,12);
				HKX61 = HKX25*P(3,10);
				HKX62 = HKX37*P(2,10);
				HKX63 = HKX25*P(3,4) - HKX33*P(4,11) + HKX35*P(4,10) + HKX36*P(4,12) + HKX37*P(2,4) - P(4,4);
				HKX64 = -(HKX25*(-HKX43 + HKX44 - HKX45 - HKX46 - HKX47 + P(3,4)) - HKX33*(-HKX53 + HKX54 - HKX55 - HKX56 - HKX57 + P(4,11)) + HKX35*(-HKX58 + HKX59 - HKX60 - HKX61 - HKX62 + P(4,10)) + HKX36*(-HKX48 + HKX49 - HKX50 - HKX51 - HKX52 + P(4,12)) + HKX37*(-HKX38 + HKX39 - HKX40 - HKX41 - HKX42 + P(2,4)) + HKX63 - obs_var(1));

				innov_var(1) = HKX64;%HKX10*HKX20 - HKX11*HKX18 + HKX12*HKX22 + HKX13*HKX16 - HKX14*HKX19 + HKX15*HKX21 + HKX17*HKX6 + HKX23 + R_MAG;

				if (innov_var(1) < obs_var(1)) 
					% the innovation variance contribution from the state covariances is negative which means the covariance matrix is badly conditioned
					fault_status.flags.bad_vel_N = true;

					% we need to re-initialise covariances and abort this fusion step
					% velocity
                    
					P(k_vel_id,k_vel_id) = sq(max(params.gps_vel_noise, 0.01));
					P(k_vel_id+1,k_vel_id+1) = P(k_vel_id,k_vel_id);
					P(k_vel_id+2,k_vel_id+2) = sq(1.5) * P(k_vel_id,k_vel_id);
					%ECL_ERR("Velocity X %s", numerical_error_covariance_reset_string);
					ret = logical(false);
                    return 
				end
				HKX64_inv = -1 / HKX64;
				fault_status.flags.bad_vel_N = false;

				test_ratio(1) = sq(innov(1)) / (sq(max(innov_gate, 1.0)) * innov_var(1));
				innov_check_fail = (test_ratio(1) > 1.0);
% 				_innov_check_fail_status.flags.reject_hor_vel = innov_check_fail;
				if(innov_check_fail) 
					ret = false;
                    return;
				end

				% Kalman gains
				Kfusion(1) = HKX64_inv*(HKX25*P(1,3) - HKX33*P(1,11) + HKX35*P(1,10) + HKX36*P(1,12) + HKX37*P(1,2) - P(1,4));
				Kfusion(2) = HKX64_inv*(HKX38 - HKX39 + HKX40 + HKX41 + HKX42 - P(2,4));
				Kfusion(3) = HKX64_inv*(HKX43 - HKX44 + HKX45 + HKX46 + HKX47 - P(3,4));
				Kfusion(4) = HKX63*HKX64_inv;
				Kfusion(5) = HKX64_inv*(HKX25*P(3,5) - HKX33*P(5,11) + HKX35*P(5,10) + HKX36*P(5,12) + HKX37*P(2,5) - P(4,5));
				Kfusion(6) = HKX64_inv*(HKX25*P(3,6) - HKX33*P(6,11) + HKX35*P(6,10) + HKX36*P(6,12) + HKX37*P(2,6) - P(4,6));
				Kfusion(7) = HKX64_inv*(HKX25*P(3,7) - HKX33*P(7,11) + HKX35*P(7,10) + HKX36*P(7,12) + HKX37*P(2,7) - P(4,7));
				Kfusion(8) = HKX64_inv*(HKX25*P(3,8) - HKX33*P(8,11) + HKX35*P(8,10) + HKX36*P(8,12) + HKX37*P(2,8) - P(4,8));
				Kfusion(9) = HKX64_inv*(HKX25*P(3,9) - HKX33*P(9,11) + HKX35*P(9,10) + HKX36*P(9,12) + HKX37*P(2,9) - P(4,9));
				Kfusion(10) = HKX64_inv*(HKX58 - HKX59 + HKX60 + HKX61 + HKX62 - P(4,10));
				Kfusion(11) = HKX64_inv*(HKX53 - HKX54 + HKX55 + HKX56 + HKX57 - P(4,11));
				Kfusion(12) = HKX64_inv*(HKX48 - HKX49 + HKX50 + HKX51 + HKX52 - P(4,12));
				Kfusion(13) = HKX64_inv*(HKX25*P(3,13) - HKX33*P(11,13) + HKX35*P(10,13) + HKX36*P(12,13) + HKX37*P(2,13) - P(4,13));
				Kfusion(14) = HKX64_inv*(HKX25*P(3,14) - HKX33*P(11,14) + HKX35*P(10,14) + HKX36*P(12,14) + HKX37*P(2,14) - P(4,14));
				Kfusion(15) = HKX64_inv*(HKX25*P(3,15) - HKX33*P(11,15) + HKX35*P(10,15) + HKX36*P(12,15) + HKX37*P(2,15) - P(4,15));
				Kfusion(16) = HKX64_inv*(HKX25*P(3,16) - HKX33*P(11,16) + HKX35*P(10,16) + HKX36*P(12,16) + HKX37*P(2,16) - P(4,16));
				Kfusion(17) = HKX64_inv*(HKX25*P(3,17) - HKX33*P(11,17) + HKX35*P(10,17) + HKX36*P(12,17) + HKX37*P(2,17) - P(4,17));
				Kfusion(18) = HKX64_inv*(HKX25*P(3,18) - HKX33*P(11,18) + HKX35*P(10,18) + HKX36*P(12,18) + HKX37*P(2,18) - P(4,18));
				Kfusion(19) = HKX64_inv*(HKX25*P(3,19) - HKX33*P(11,19) + HKX35*P(10,19) + HKX36*P(12,19) + HKX37*P(2,19) - P(4,19));
				Kfusion(20) = HKX64_inv*(HKX25*P(3,20) - HKX33*P(11,20) + HKX35*P(10,20) + HKX36*P(12,20) + HKX37*P(2,20) - P(4,20));
				Kfusion(21) = HKX64_inv*(HKX25*P(3,21) - HKX33*P(11,21) + HKX35*P(10,21) + HKX36*P(12,21) + HKX37*P(2,21) - P(4,21));
				Kfusion(22) = HKX64_inv*(HKX25*P(3,22) - HKX33*P(11,22) + HKX35*P(10,22) + HKX36*P(12,22) + HKX37*P(2,22) - P(4,22));
				Kfusion(23) = HKX64_inv*(HKX25*P(3,23) - HKX33*P(11,23) + HKX35*P(10,23) + HKX36*P(12,23) + HKX37*P(2,23) - P(4,23));

			
			
			case 2
			
				% Axis 2 equations
				% Sub Expressions
				HKY0 = q1*q2;
				HKY1 = q3*q4;
				HKY2 = HKY0 + HKY1;
				HKY3 = q2*q4;
				HKY4 = q1*q3;
				HKY5 = 2*z_offset_body;
				HKY6 = 2*q2*q2;
				HKY7 = -HKY6;
				HKY8 = 2*q3*q3;
				HKY9 = 1 - HKY8;
				HKY10 = HKY8 - 1;
				HKY11 = -gyro_unbias(1)*(-HKY2*HKY5 + y_offset_body*(HKY7 + HKY9)) + gyro_unbias(2)*(HKY5*(-HKY3 + HKY4) - x_offset_body*(HKY10 + HKY6)) - 3*gyro_unbias(3)*(HKY2*x_offset_body - y_offset_body*(HKY3 - HKY4));
				HKY12 = HKY3 + HKY4;
				HKY13 = HKY12*y_offset_body;
				HKY14 = q2*q3;
				HKY15 = q1*q4;
				HKY16 = HKY14 - HKY15;
				HKY17 = HKY16*z_offset_body;
				HKY18 = 2*gyro_unbias(1);
				HKY19 = 2*x_offset_body;
				HKY20 = HKY12*HKY19;
				HKY21 = 2*q4*q4;
				HKY22 = HKY16*HKY19;
				HKY23 = -HKY21;
				HKY24 = HKY23 + HKY9;
				HKY25 = HKY24*y_offset_body;
				HKY26 = 1.0/dt;
				HKY27 = HKY0 - HKY1;
				HKY28 = 2*y_offset_body;
				HKY29 = HKY21 + HKY6 - 1;
				HKY30 = HKY14 + HKY15;
				HKY31 = 2*HKY26*(HKY27*x_offset_body + HKY30*z_offset_body);
				HKY32 = HKY28*HKY30;
				HKY33 = HKY31*P(1,11);
				HKY34 = HKY23 + HKY7 + 1;
				HKY35 = HKY26*(-HKY32 + HKY34*x_offset_body);
				HKY36 = HKY35*P(1,12);
				HKY37 = HKY26*(HKY28*(-HKY0 + HKY1) - HKY34*z_offset_body);
				HKY38 = HKY37*P(1,10);
				HKY39 = HKY18*(-HKY13 + HKY17) + gyro_unbias(2)*(HKY20 - HKY24*z_offset_body) + gyro_unbias(3)*(-HKY22 + HKY25);
				HKY40 = HKY39*P(1,3);
				HKY41 = HKY11*P(1,1);
				HKY42 = HKY31*P(3,11);
				HKY43 = HKY35*P(3,12);
				HKY44 = HKY37*P(3,10);
				HKY45 = HKY11*P(1,3);
				HKY46 = HKY39*P(3,3);
				HKY47 = HKY31*P(11,12);
				HKY48 = HKY35*P(12,12);
				HKY49 = HKY37*P(10,12);
				HKY50 = HKY11*P(1,12);
				HKY51 = HKY39*P(3,12);
				HKY52 = HKY31*P(10,11);
				HKY53 = HKY35*P(10,12);
				HKY54 = HKY37*P(10,10);
				HKY55 = HKY11*P(1,10);
				HKY56 = HKY39*P(3,10);
				HKY57 = HKY31*P(11,11);
				HKY58 = HKY35*P(11,12);
				HKY59 = HKY37*P(10,11);
				HKY60 = HKY11*P(1,11);
				HKY61 = HKY39*P(3,11);
				HKY62 = -HKY11*P(1,5) + HKY31*P(5,11) + HKY35*P(5,12) + HKY37*P(5,10) + HKY39*P(3,5) - P(5,5);
				HKY63 = -(-HKY11*(-HKY33 - HKY36 - HKY38 - HKY40 + HKY41 + P(1,5)) + HKY31*(-HKY57 - HKY58 - HKY59 + HKY60 - HKY61 + P(5,11)) + HKY35*(-HKY47 - HKY48 - HKY49 + HKY50 - HKY51 + P(5,12)) + HKY37*(-HKY52 - HKY53 - HKY54 + HKY55 - HKY56 + P(5,10)) + HKY39*(-HKY42 - HKY43 - HKY44 + HKY45 - HKY46 + P(3,5)) + HKY62 - obs_var(2));


				innov_var(2) = HKY63;%HKX10*HKX20 - HKX11*HKX18 + HKX12*HKX22 + HKX13*HKX16 - HKX14*HKX19 + HKX15*HKX21 + HKX17*HKX6 + HKX23 + R_MAG;

				if (innov_var(2) < obs_var(2)) 
					% the innovation variance contribution from the state covariances is negative which means the covariance matrix is badly conditioned
					fault_status.flags.bad_vel_E = true;

					% we need to re-initialise covariances and abort this fusion step
					% velocity
					P(k_vel_id,k_vel_id) = sq(max(params.gps_vel_noise, 0.01));
					P(k_vel_id+1,k_vel_id+1) = P(k_vel_id,k_vel_id);
					P(k_vel_id+2,k_vel_id+2) = sq(1.5) * P(k_vel_id,k_vel_id);
					%ECL_ERR("Velocity Y %s", numerical_error_covariance_reset_string);
					ret = false;
                    return;
				end
				HKY63_inv = -1.0 / HKY63;
				fault_status.flags.bad_vel_E = false;

				test_ratio(2) = sq(innov(2)) / (sq(max(innov_gate, 1)) * innov_var(2));
				innov_check_fail = (test_ratio(2) > 1);
				%_innov_check_fail_status.flags.reject_hor_vel = innov_check_fail;
				if(innov_check_fail) 
					ret =  false;
                    return;
				end


                Kfusion(1) = HKY63_inv*(HKY33 + HKY36 + HKY38 + HKY40 - HKY41 - P(1,5));
				Kfusion(2) = HKY63_inv*(-HKY11*P(1,2) + HKY31*P(2,11) + HKY35*P(2,12) + HKY37*P(2,10) + HKY39*P(2,3) - P(2,5));
				Kfusion(3) = HKY63_inv*(HKY42 + HKY43 + HKY44 - HKY45 + HKY46 - P(3,5));
				Kfusion(4) = HKY63_inv*(-HKY11*P(1,4) + HKY31*P(4,11) + HKY35*P(4,12) + HKY37*P(4,10) + HKY39*P(3,4) - P(4,5));
				Kfusion(5) = HKY62*HKY63_inv;
				Kfusion(6) = HKY63_inv*(-HKY11*P(1,6) + HKY31*P(6,11) + HKY35*P(6,12) + HKY37*P(6,10) + HKY39*P(3,6) - P(5,6));
				Kfusion(7) = HKY63_inv*(-HKY11*P(1,7) + HKY31*P(7,11) + HKY35*P(7,12) + HKY37*P(7,10) + HKY39*P(3,7) - P(5,7));
				Kfusion(8) = HKY63_inv*(-HKY11*P(1,8) + HKY31*P(8,11) + HKY35*P(8,12) + HKY37*P(8,10) + HKY39*P(3,8) - P(5,8));
				Kfusion(9) = HKY63_inv*(-HKY11*P(1,9) + HKY31*P(9,11) + HKY35*P(9,12) + HKY37*P(9,10) + HKY39*P(3,9) - P(5,9));
				Kfusion(10) = HKY63_inv*(HKY52 + HKY53 + HKY54 - HKY55 + HKY56 - P(5,10));
				Kfusion(11) = HKY63_inv*(HKY57 + HKY58 + HKY59 - HKY60 + HKY61 - P(5,11));
				Kfusion(12) = HKY63_inv*(HKY47 + HKY48 + HKY49 - HKY50 + HKY51 - P(5,12));
				Kfusion(13) = HKY63_inv*(-HKY11*P(1,13) + HKY31*P(11,13) + HKY35*P(12,13) + HKY37*P(10,13) + HKY39*P(3,13) - P(5,13));
				Kfusion(14) = HKY63_inv*(-HKY11*P(1,14) + HKY31*P(11,14) + HKY35*P(12,14) + HKY37*P(10,14) + HKY39*P(3,14) - P(5,14));
				Kfusion(15) = HKY63_inv*(-HKY11*P(1,15) + HKY31*P(11,15) + HKY35*P(12,15) + HKY37*P(10,15) + HKY39*P(3,15) - P(5,15));
				Kfusion(16) = HKY63_inv*(-HKY11*P(1,16) + HKY31*P(11,16) + HKY35*P(12,16) + HKY37*P(10,16) + HKY39*P(3,16) - P(5,16));
				Kfusion(17) = HKY63_inv*(-HKY11*P(1,17) + HKY31*P(11,17) + HKY35*P(12,17) + HKY37*P(10,17) + HKY39*P(3,17) - P(5,17));
				Kfusion(18) = HKY63_inv*(-HKY11*P(1,18) + HKY31*P(11,18) + HKY35*P(12,18) + HKY37*P(10,18) + HKY39*P(3,18) - P(5,18));
				Kfusion(19) = HKY63_inv*(-HKY11*P(1,19) + HKY31*P(11,19) + HKY35*P(12,19) + HKY37*P(10,19) + HKY39*P(3,19) - P(5,19));
				Kfusion(20) = HKY63_inv*(-HKY11*P(1,20) + HKY31*P(11,20) + HKY35*P(12,20) + HKY37*P(10,20) + HKY39*P(3,20) - P(5,20));
				Kfusion(21) = HKY63_inv*(-HKY11*P(1,21) + HKY31*P(11,21) + HKY35*P(12,21) + HKY37*P(10,21) + HKY39*P(3,21) - P(5,21));
				Kfusion(22) = HKY63_inv*(-HKY11*P(1,22) + HKY31*P(11,22) + HKY35*P(12,22) + HKY37*P(10,22) + HKY39*P(3,22) - P(5,22));
				Kfusion(23) = HKY63_inv*(-HKY11*P(1,23) + HKY31*P(11,23) + HKY35*P(12,23) + HKY37*P(10,23) + HKY39*P(3,23) - P(5,23));

            case 3

                % Axis 2 equations
				% Sub Expressions
				HKZ0 = q1*q4;
				HKZ1 = q2*q3;
				HKZ2 = HKZ0 + HKZ1;
				HKZ3 = HKZ2*z_offset_body;
				HKZ4 = q1*q2;
				HKZ5 = q3*q4;
				HKZ6 = 2*gyro_unbias(2);
				HKZ7 = 2*y_offset_body;
				HKZ8 = HKZ2*HKZ7;
				HKZ9 = 2*q2*q2;
				HKZ10 = -HKZ9;
				HKZ11 = 2*q4*q4;
				HKZ12 = 1 - HKZ11;
				HKZ13 = HKZ10 + HKZ12;
				HKZ14 = HKZ13*x_offset_body;
				HKZ15 = -HKZ4 + HKZ5;
				HKZ16 = HKZ15*HKZ7;
				HKZ17 = HKZ13*z_offset_body;
				HKZ18 = HKZ6*(HKZ3 + x_offset_body*(HKZ4 - HKZ5)) + gyro_unbias(1)*(HKZ16 - HKZ17) + gyro_unbias(3)*(HKZ14 - HKZ8);
				HKZ19 = q1*q3;
				HKZ20 = q2*q4;
				HKZ21 = HKZ19 + HKZ20;
				HKZ22 = -HKZ0 + HKZ1;
				HKZ23 = 2*gyro_unbias(1)*(HKZ21*y_offset_body - HKZ22*z_offset_body);
				HKZ24 = 2*x_offset_body;
				HKZ25 = 2*q3*q3;
				HKZ26 = HKZ25 - 1;
				HKZ27 = gyro_unbias(2)*(HKZ21*HKZ24 + z_offset_body*(HKZ11 + HKZ26));
				HKZ28 = -HKZ25;
				HKZ29 = gyro_unbias(3)*(HKZ22*HKZ24 - y_offset_body*(HKZ12 + HKZ28));
				HKZ30 = -HKZ23 + HKZ27 - HKZ29;
				HKZ31 = 1/dt;
				HKZ32 = HKZ4 + HKZ5;
				HKZ33 = 2*z_offset_body;
				HKZ34 = HKZ32*HKZ33;
				HKZ35 = HKZ26 + HKZ9;
				HKZ36 = HKZ31*(HKZ34 + HKZ35*y_offset_body);
				HKZ37 = HKZ19 - HKZ20;
				HKZ38 = HKZ33*HKZ37;
				HKZ39 = HKZ35*x_offset_body;
				HKZ40 = HKZ31*(-HKZ38 + HKZ39);
				HKZ41 = HKZ32*x_offset_body;
				HKZ42 = 2*HKZ31;
				HKZ43 = HKZ42*(HKZ37*y_offset_body + HKZ41);
				HKZ44 = HKZ42*(HKZ41 - y_offset_body*(-HKZ19 + HKZ20));
				HKZ45 = HKZ31*(-HKZ34 + y_offset_body*(HKZ10 + HKZ28 + 1));
				HKZ46 = HKZ31*(HKZ38 - HKZ39);
				HKZ47 = HKZ6*(HKZ15*x_offset_body - HKZ3) + gyro_unbias(1)*(-HKZ16 + HKZ17) + gyro_unbias(3)*(-HKZ14 + HKZ8);
				HKZ48 = -HKZ30*P(1,2) + HKZ44*P(1,12) + HKZ45*P(1,10) - HKZ46*P(1,11) + HKZ47*P(1,1) - P(1,6);
				HKZ49 = HKZ23 - HKZ27 + HKZ29;
				HKZ50 = -HKZ30*P(2,12) + HKZ44*P(12,12) + HKZ45*P(10,12) - HKZ46*P(11,12) + HKZ47*P(1,12) - P(6,12);
				HKZ51 = -HKZ30*P(2,11) + HKZ44*P(11,12) + HKZ45*P(10,11) - HKZ46*P(11,11) + HKZ47*P(1,11) - P(6,11);
				HKZ52 = -HKZ30*P(2,10) + HKZ44*P(10,12) + HKZ45*P(10,10) - HKZ46*P(10,11) + HKZ47*P(1,10) - P(6,10);
				HKZ53 = -HKZ30*P(2,2) + HKZ44*P(2,12) + HKZ45*P(2,10) - HKZ46*P(2,11) + HKZ47*P(1,2) - P(2,6);
				HKZ54 = (-HKZ18*HKZ48 + HKZ18*P(1,6) - HKZ36*HKZ52 + HKZ36*P(6,10) + HKZ40*HKZ51 - HKZ40*P(6,11) + HKZ43*HKZ50 - HKZ43*P(6,12) + HKZ49*HKZ53 - HKZ49*P(2,6) + obs_var(3) + P(6,6));

				innov_var(3) = HKZ54;%HKX10*HKX20 - HKX11*HKX18 + HKX12*HKX22 + HKX13*HKX16 - HKX14*HKX19 + HKX15*HKX21 + HKX17*HKX6 + HKX23 + R_MAG;

				if (innov_var(3) < obs_var(3)) 
					% the innovation variance contribution from the state covariances is negative which means the covariance matrix is badly conditioned
					fault_status.flags.bad_vel_D = true;

					% we need to re-initialise covariances and abort this fusion step
					% velocity
					P(k_vel_id,k_vel_id) = sq(max(params.gps_vel_noise, 0.01));
					P(k_vel_id+1,k_vel_id+1) = P(k_vel_id,k_vel_id);
					P(k_vel_id+2,k_vel_id+2) = sq(1.5) * P(k_vel_id,k_vel_id);
					%ECL_ERR("Velocity Z %s", numerical_error_covariance_reset_string);
					ret = false;
                    return 
				end
				HKZ54_inv = 1 / HKZ54;
				fault_status.flags.bad_vel_D = false;

				test_ratio(3) = sq(innov(3)) / (sq(max(innov_gate, 1)) * innov_var(3));
				innov_check_fail = (test_ratio(3) > 1.0);
				%_innov_check_fail_status.flags.reject_ver_vel = innov_check_fail;
				if(innov_check_fail) 
					ret =  false;
                    return
				end

				% Kalman gains
				Kfusion(1) = -HKZ48*HKZ54_inv;
				Kfusion(2) = -HKZ53*HKZ54_inv;
				Kfusion(3) = -HKZ54_inv*(-HKZ30*P(2,3) + HKZ44*P(3,12) + HKZ45*P(3,10) - HKZ46*P(3,11) + HKZ47*P(1,3) - P(3,6));
				Kfusion(4) = -HKZ54_inv*(-HKZ30*P(2,4) + HKZ44*P(4,12) + HKZ45*P(4,10) - HKZ46*P(4,11) + HKZ47*P(1,4) - P(4,6));
				Kfusion(5) = -HKZ54_inv*(-HKZ30*P(2,5) + HKZ44*P(5,12) + HKZ45*P(5,10) - HKZ46*P(5,11) + HKZ47*P(1,5) - P(5,6));
				Kfusion(6) = -HKZ54_inv*(-HKZ30*P(2,6) + HKZ44*P(6,12) + HKZ45*P(6,10) - HKZ46*P(6,11) + HKZ47*P(1,6) - P(6,6));
				Kfusion(7) = -HKZ54_inv*(-HKZ30*P(2,7) + HKZ44*P(7,12) + HKZ45*P(7,10) - HKZ46*P(7,11) + HKZ47*P(1,7) - P(6,7));
				Kfusion(8) = -HKZ54_inv*(-HKZ30*P(2,8) + HKZ44*P(8,12) + HKZ45*P(8,10) - HKZ46*P(8,11) + HKZ47*P(1,8) - P(6,8));
				Kfusion(9) = -HKZ54_inv*(-HKZ30*P(2,9) + HKZ44*P(9,12) + HKZ45*P(9,10) - HKZ46*P(9,11) + HKZ47*P(1,9) - P(6,9));
				Kfusion(10) = -HKZ52*HKZ54_inv;
				Kfusion(11) = -HKZ51*HKZ54_inv;
				Kfusion(12) = -HKZ50*HKZ54_inv;
				Kfusion(13) = -HKZ54_inv*(-HKZ30*P(2,13) + HKZ44*P(12,13) + HKZ45*P(10,13) - HKZ46*P(11,13) + HKZ47*P(1,13) - P(6,13));
				Kfusion(14) = -HKZ54_inv*(-HKZ30*P(2,14) + HKZ44*P(12,14) + HKZ45*P(10,14) - HKZ46*P(11,14) + HKZ47*P(1,14) - P(6,14));
				Kfusion(15) = -HKZ54_inv*(-HKZ30*P(2,15) + HKZ44*P(12,15) + HKZ45*P(10,15) - HKZ46*P(11,15) + HKZ47*P(1,15) - P(6,15));
				Kfusion(16) = -HKZ54_inv*(-HKZ30*P(2,16) + HKZ44*P(12,16) + HKZ45*P(10,16) - HKZ46*P(11,16) + HKZ47*P(1,16) - P(6,16));
				Kfusion(17) = -HKZ54_inv*(-HKZ30*P(2,17) + HKZ44*P(12,17) + HKZ45*P(10,17) - HKZ46*P(11,17) + HKZ47*P(1,17) - P(6,17));
				Kfusion(18) = -HKZ54_inv*(-HKZ30*P(2,18) + HKZ44*P(12,18) + HKZ45*P(10,18) - HKZ46*P(11,18) + HKZ47*P(1,18) - P(6,18));
				Kfusion(19) = -HKZ54_inv*(-HKZ30*P(2,19) + HKZ44*P(12,19) + HKZ45*P(10,19) - HKZ46*P(11,19) + HKZ47*P(1,19) - P(6,19));
				Kfusion(20) = -HKZ54_inv*(-HKZ30*P(2,20) + HKZ44*P(12,20) + HKZ45*P(10,20) - HKZ46*P(11,20) + HKZ47*P(1,20) - P(6,20));
				Kfusion(21) = -HKZ54_inv*(-HKZ30*P(2,21) + HKZ44*P(12,21) + HKZ45*P(10,21) - HKZ46*P(11,21) + HKZ47*P(1,21) - P(6,21));
				Kfusion(22) = -HKZ54_inv*(-HKZ30*P(2,22) + HKZ44*P(12,22) + HKZ45*P(10,22) - HKZ46*P(11,22) + HKZ47*P(1,22) - P(6,22));
				Kfusion(23) = -HKZ54_inv*(-HKZ30*P(2,23) + HKZ44*P(12,23) + HKZ45*P(10,23) - HKZ46*P(11,23) + HKZ47*P(1,23) - P(6,23));

        end
			
% 	    deltastates = zeros(23,1);
        assignin("base","Kfusion"+num2str(3+index),Kfusion);
	    is_fused = measurementUpdate(Kfusion, innov_var(index), innov(index));
    
    
% 	    switch (index) 
% 	    case 1
% 		    fault_status.flags.bad_vel_N = !is_fused;
% 		    _velx_deltastates = deltastates;
% 		    
%     
% 	    case 2
% 		    fault_status.flags.bad_vel_E = !is_fused;
% 		    _vely_deltastates = deltastates;
% 		    
% 	    case 3
% 		    fault_status.flags.bad_vel_D = !is_fused;
% 		    _velz_deltastates = deltastates;
% 		    
% 	    end
    
% 	    if(is_fused) 
% 		    if(index == 3) 
% 			    time_last_ver_vel_fuse = time_last_imu;
%             elseif(index == 1 || index == 2) 
% 			    time_last_hor_vel_fuse = time_last_imu;
% 		    end
% 	    end
	    ret = ret && is_fused;
    end
    
    
end
    