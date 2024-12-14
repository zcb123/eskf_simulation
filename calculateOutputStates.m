function  calculateOutputStates(imu,correct_updated)
 
    global params CONSTANTS_ONE_G
    global states imu_sample_delayed dt_imu_avg  dt_ekf_avg;        %这里只是使用了states,没有赋值
    global yaw_delta_ef R_to_earth_now;
    
    global output_buffer output_new ;
 
    global delta_angle_corr;
    global yaw_rate_lpf_ef;

    persistent vel_err_integ;
    persistent pos_err_integ;
   
    
    if isempty(vel_err_integ)
        vel_err_integ = single([0 0 0]');
    end
    if isempty(pos_err_integ)
        pos_err_integ = single([0 0 0]');
    end
    
    
	dt_scale_correction = dt_imu_avg / dt_ekf_avg;
	delta_angle = imu.delta_ang - states.delta_ang_bias*dt_scale_correction + delta_angle_corr;
    assignin("base","delta_angle",delta_angle);
  
    spin_del_ang_D = sum(delta_angle.*R_to_earth_now(3,:)');
    yaw_delta_ef = yaw_delta_ef + spin_del_ang_D;

    yaw_rate_lpf_ef = 0.95*yaw_rate_lpf_ef + 0.05*spin_del_ang_D / imu.delta_ang_dt;

	output_new.time_us = imu.time_us;

	dq = Quaternion_from_AxisAngle_3arg(delta_angle);
    assignin("base","dq",dq);

	output_new.quat_nominal = quatMult(output_new.quat_nominal,dq);
	output_new.quat_nominal = quat_normalize(output_new.quat_nominal);
	R_to_earth_now = Quat2Tbn(output_new.quat_nominal);
 
	delta_vel_body = imu.delta_vel - states.delta_vel_bias * dt_scale_correction;
    assignin("base","dt_scale_correction",dt_scale_correction);
    assignin("base","delta_vel_body",delta_vel_body);
	delta_vel_earth = R_to_earth_now * delta_vel_body;
	delta_vel_earth(3) = delta_vel_earth(3) + CONSTANTS_ONE_G * imu.delta_vel_dt;
    assignin("base","delta_vel_earth",delta_vel_earth);
% 	if imu.delta_vel_dt > 1e-4 
% 		vel_deriv = delta_vel_earth * (1.0 / imu.delta_vel_dt);
%     end     
	
	vel_last = output_new.vel;

	output_new.vel = output_new.vel + delta_vel_earth;

	delta_pos_NED = (output_new.vel + vel_last) * (imu.delta_vel_dt * 0.5);
	output_new.pos = output_new.pos + delta_pos_NED;
	assignin('base','output_new_pos',output_new.pos);

	if imu.delta_ang_dt>1e-4			
		ang_rate = imu.delta_ang  / imu.delta_ang_dt;		
		vel_imu_rel_body = cross(ang_rate, params.imu_pos_body);	
		vel_imu_rel_body_ned = R_to_earth_now * vel_imu_rel_body;	
    end

	if (correct_updated)

        output_buffer.push(output_new);
		output_delayed = output_buffer.get_oldest();
               
        quat_nominal_inverse = Quaternion_Inverse(states.quat_nominal);
        assignin('base',"quat_nominal_inverse",quat_nominal_inverse);
        %新旧数据的补偿
        quat_delta_delay = quatMult(quat_nominal_inverse,output_delayed.quat_nominal);
		q_error = quat_normalize(quat_delta_delay);
        assignin("base",'quat_delta_delay',quat_delta_delay);
       
        if q_error(1)>=single(0)
            scalar = -2;
        else
            scalar = 2;
        end		

		delta_ang_error = [scalar*q_error(2) scalar*q_error(3) scalar*q_error(4)]';
		time_delay = max(single((imu.time_us - imu_sample_delayed.time_us)) * single(1e-6), dt_imu_avg);
		att_gain = single(0.5) * dt_imu_avg / time_delay;

		delta_angle_corr = delta_ang_error * att_gain;
		
		vel_gain = dt_ekf_avg / saturation(params.vel_Tau, dt_ekf_avg, single(10.0));
		pos_gain = dt_ekf_avg / saturation(params.pos_Tau, dt_ekf_avg, single(10.0));
        %新旧数据的补偿
		vel_err = states.vel - output_delayed.vel;
		pos_err = states.pos - output_delayed.pos;
        %系数和补偿的方式还得再看看论文
		vel_err_integ = vel_err_integ + vel_err;
		vel_correction = vel_err * vel_gain + vel_err_integ * (vel_gain^2) * 0.1;

		
		pos_err_integ = pos_err_integ + pos_err;
		pos_correction = pos_err * pos_gain + pos_err_integ * (pos_gain^2) * 0.1;

		
        applyCorrectionToOutputBuffer(vel_correction,pos_correction);
        assignin("base","vel_correction",vel_correction);
        assignin("base","pos_correction",pos_correction);

    end
    
    %最后输出时校正
    delta_angle = imu.delta_ang - states.delta_ang_bias * (dt_imu_avg / dt_ekf_avg) + delta_angle_corr;
    delta_q = Quaternion_from_AxisAngle_3arg(delta_angle);
    updated_q = QuatMult(output_new.quat_nominal,delta_q);
    output_new.quat_nominal = quat_normalize(updated_q);
    
    
    
    output_new.vel = output_new.vel - vel_imu_rel_body_ned;     %这里的值等于零
    output_new.pos = output_new.pos - R_to_earth_now*params.imu_pos_body;

end