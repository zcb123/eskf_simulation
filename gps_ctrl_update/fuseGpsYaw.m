function is_fused = fuseGpsYaw(gps_sample_delayed,params,control_status)
    global states P dt_ekf_avg R_to_earth fault_status;
    global gps_yaw_offset;

    q1 = states.quat_nominal(1);
	q2 = states.quat_nominal(2);
	q3 = states.quat_nominal(3);
	q4 = states.quat_nominal(4);

	% calculate the observed yaw angle of antenna array, converting a from body to antenna yaw measurement
	measured_hdg = wrap_pn_pi(gps_sample_delayed.yaw + gps_yaw_offset);

	% define the predicted antenna array vector and rotate into earth frame
	ant_vec_bf = [cos(gps_yaw_offset) sin(gps_yaw_offset) 0]';
	ant_vec_ef = R_to_earth * ant_vec_bf;

	% check if antenna array vector is within 30 degrees of vertical and therefore unable to provide a reliable heading
    % 检查天线阵列矢量是否在垂直方向30度以内，因此无法提供可靠的航向
    % cos(pi/6)相当于sin(pi/3).两个天线点的连线与北东平面的夹角不能超过60度(即两个天线的高度差不能太大) 
    % 现在这里报错说明R_to_earth与实际的角度相差甚大.因为实际的天线连线投影是满足要求的
	if abs(ant_vec_ef(3)) > cos(30/57.3)    
        disp('ant_vec_ef too large');
		return;
	end

	% calculate predicted antenna yaw angle
	predicted_hdg = atan2(ant_vec_ef(2), ant_vec_ef(1));

	% using magnetic heading process noise
	% TODO extend interface to use yaw uncertainty provided by GPS if available
	R_YAW = sq(max(params.gps_heading_noise, 1.0e-2));

	% calculate intermediate variables
	SA0 = ant_vec_bf(2);%sinf(ant_yaw);
	SA1 = 2*q1;
	SA2 = SA1*q4;
	SA3 = 2*q2;
	SA4 = SA3*q3;
	SA5 = SA0*(-SA2 + SA4);
	SA6 = ant_vec_bf(1);%cosf(ant_yaw);
	SA7 = 1 - 2*powf(q4, 2);
	SA8 = SA6*(SA7 - 2*powf(q3, 2));
	SA9 = SA6*(SA2 + SA4);
	SA10 = SA0*(SA7 - 2*powf(q2, 2));
	SA11 = (powf(SA10 + SA9, 2) + powf(SA5 + SA8, 2));
% 	SA11_inv;

	if SA11 < 1e-6
		SA11_inv = 0;
	else 
		SA11_inv = (SA0*(SA1*q2 + 2*q3*q4) + SA6*(-SA1*q3 + SA3*q4)) / SA11;
	end


	HYaw = zeros(23,1);
	HYaw(1) = SA11_inv*(-SA5 - SA8);
	HYaw(2) = SA11_inv*(-SA10 - SA9);
	HYaw(3) = 1;

	% check if the innovation variance calculation is badly conditioned
	% calculate the innovation variance
	heading_innov_var = R_YAW;
    %P*HYaw
	tmp1 = P(1, 1) * HYaw(1) + P(1, 2) * HYaw(2) + P(1, 3) * HYaw(3);
	tmp2 = P(2, 1) * HYaw(1) + P(2, 2) * HYaw(2) + P(2, 3) * HYaw(3);
	tmp3 = P(3, 1) * HYaw(1) + P(3, 2) * HYaw(2) + P(3, 3) * HYaw(3);
	heading_innov_var = heading_innov_var + tmp1*HYaw(1) + tmp2*HYaw(2) + tmp3*HYaw(3);   


	if heading_innov_var < R_YAW    %这里是个平方和，必然大于R_YAW
		% the innovation variance contribution from the state covariances is negative which means the covariance matrix is badly conditioned
		fault_status.flags.bad_hdg = true;
        disp('bad heading_innov_var');
		
		initialiseCovariance();
		return;
	end
	heading_innov_var_inv = 1 / heading_innov_var;

	%fault_status.flags.bad_hdg = false;

	% calculate the innovation and define the innovation gate
	innov_gate = max(params.heading_innov_gate, 1);
	heading_innov = predicted_hdg - measured_hdg;

	% wrap the innovation to the interval between +-pi
	heading_innov = wrap_pn_pi(heading_innov);
    
    global yaw_test_ratio;
	% innovation test ratio
	yaw_test_ratio = sq(heading_innov) / (sq(innov_gate) * heading_innov_var);

	% we are no longer using 4-axis fusion so set the reported test levels to zero
	%_mag_test_ratio.setZero();

	if (yaw_test_ratio > 1.0) 
		%innov_check_fail_status.flags.reject_yaw = true;
        disp('reject gps yaw fuse yaw_test_ratio>1');
		return;

	else 
		%innov_check_fail_status.flags.reject_yaw = false;
	end
    global yaw_signed_test_ratio_lpf;
    yaw_signed_test_ratio_lpf.update(sign(heading_innov) * yaw_test_ratio);
    yaw_getState = yaw_signed_test_ratio_lpf.getState();
	if (~control_status.flags.in_air...
	    && abs(yaw_getState) > 0.2) 

		% A constant large signed test ratio is a sign of wrong gyro bias
		% Reset the yaw gyro variance to converge faster and avoid
		% being stuck on a previous bad estimate
		resetZDeltaAngBiasCov();

	end

	% calculate the Kalman gains
	% only calculate gains for states we are using
	Kfusion = zeros(23,1);
	for row = 1:23 
		
		Kfusion(row)  = P(row, 1) * HYaw(1);
		Kfusion(row)  = Kfusion(row) + P(row, 2) * HYaw(2);
		Kfusion(row)  = Kfusion(row) + P(row, 3) * HYaw(3);

		Kfusion(row) = Kfusion(row)*heading_innov_var_inv;

%         if row == 11
%             assignin("base","Kfusionyaw"+num2str(row),Kfusion(row))
%         end

	end
    assignin("base","Kfusion_gps_yaw",Kfusion);
	is_fused = measurementUpdate(Kfusion, heading_innov_var, heading_innov);

	fault_status.flags.bad_hdg = ~is_fused;
    global time_last_gps_yaw_fuse time_last_imu;
    
	if (is_fused) 
		time_last_gps_yaw_fuse = time_last_imu;           
	end




end



