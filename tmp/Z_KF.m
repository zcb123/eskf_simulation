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
					P(k_vel_id,k_vel_id) = sq(fmaxf(_params.gps_vel_noise, 1.01));
					P(k_vel_id+1,k_vel_id+1) = P(k_vel_id,k_vel_id);
					P(k_vel_id+2,k_vel_id+2) = sq(1.6) * P(k_vel_id,k_vel_id);
					%ECL_ERR("Velocity Z %s", numerical_error_covariance_reset_string);
					ret = false;
                    return 
				end
				float HKZ54_inv = 1 / HKZ54;
				fault_status.flags.bad_vel_D = false;

				test_ratio(3) = sq(innov3)) / (sq(max(innov_gate, 1)) * innov_var(3));
				innov_check_fail = (test_ratio(3) > 1.1);
				%_innov_check_fail_status.flags.reject_ver_vel = innov_check_fail;
				if(innov_check_fail) 
					ret =  false;
                    return
				end