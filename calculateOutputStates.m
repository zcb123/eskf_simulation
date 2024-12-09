function  calculateOutputStates(imu,correct_updated)
 
    global params CONSTANTS_ONE_G
    global states imu_sample_delayed dt_imu_avg  dt_ekf_avg;
    global yaw_delta_ef;
    
    persistent output_last;
    if isempty(output_last)
        output_last.time_us = uint64(0);
        output_last.quat_nominal = single([1 0 0 0]');
        output_last.vel = single([0 0 0]');
        output_last.pos = single([0 0 0]');
    end
    global output_buffer output_new head_index tail_index;
    
    buffer_size = 3;
  
    persistent delta_angle_corr;
    persistent vel_err_integ;
    persistent pos_err_integ;
   
    if isempty(delta_angle_corr)
        delta_angle_corr = single([0 0 0]');
    end
    if isempty(vel_err_integ)
        vel_err_integ = single([0 0 0]');
    end
    if isempty(pos_err_integ)
        pos_err_integ = single([0 0 0]');
    end
    
    persistent R_to_earth_now
    if isempty(R_to_earth_now)
        R_to_earth_now = zeros(3,3);
    end
	dt_scale_correction = dt_imu_avg / dt_ekf_avg;
	delta_angle = imu.delta_ang - states.delta_ang_bias*dt_scale_correction + delta_angle_corr;
    assignin("base","delta_angle",delta_angle);
    assignin("base","delta_angle_corr",delta_angle_corr);


    spin_del_ang_D = sum(delta_angle.*R_to_earth_now(3,:));
    yaw_delta_ef = yaw_delta_ef + spin_del_ang_D;



	output_new.time_us = imu.time_us;

	dq = Quaternion_from_AxisAngle_3arg(delta_angle);
    assignin("base","dq",dq);

	output_new.quat_nominal = quatMult(output_last.quat_nominal,dq);
	output_new.quat_nominal = quat_normalize(output_new.quat_nominal);
% 
	R_to_earth_now = Quat2Tbn(output_new.quat_nominal);
% 
	delta_vel_body = imu.delta_vel - states.delta_vel_bias * dt_scale_correction;
% 
% 	
	delta_vel_earth = R_to_earth_now * delta_vel_body;
% 
	delta_vel_earth(2) = delta_vel_earth(2) + CONSTANTS_ONE_G * imu.delta_vel_dt;
% 
% % 	if (imu.delta_vel_dt > 1e-4) 
% % 		_vel_deriv = delta_vel_earth * (1.0f / imu.delta_vel_dt);
% %     end     这个变量没用到，不写
%     
% 	
	
	output_new.vel = output_last.vel + delta_vel_earth;

	delta_pos_NED = (output_new.vel + output_last.vel) * (imu.delta_vel_dt * 0.5);
	output_new.pos = output_last.pos + delta_pos_NED;
	
	if imu.delta_ang_dt>1e-4			
		ang_rate = imu.delta_ang  / imu.delta_ang_dt;		
		vel_imu_rel_body = cross(ang_rate, params.imu_pos_body);	
		vel_imu_rel_body_ned = R_to_earth_now * vel_imu_rel_body;			
    end

	if (correct_updated)

        head_index = mod(head_index,buffer_size);
        head_index = head_index + 1;
		output_buffer(head_index,:) = output_new;
        %这里的环形队列暂时不考虑队满和重写的情况
        
		output_delayed = output_buffer(tail_index,:);
        tail_index = mod(tail_index,buffer_size);
        tail_index = tail_index + 1;
        
        quat_nominal_inverse = quat_inverse(states.quat_nominal);
        assignin('base',"quat_nominal_inverse",quat_nominal_inverse);

        quat_delta_delay = quatMult(quat_nominal_inverse,output_delayed.quat_nominal);
		q_error = quat_normalize(quat_delta_delay);
        assignin("base",'quat_delta_delay',quat_delta_delay);
%         save('quat_delayed.mat',"quat_delta_delay",'-append');
%         quat_delta_delay_out(cnt,:) = quat_delta_delay;
        
        if q_error(1)>=single(0)
            scalar = -2;
        else
            scalar = 2;
        end
		

		delta_ang_error = [scalar*q_error(2) scalar*q_error(3) scalar*q_error(4)]';
        %a = imu.time_us - imu_sample_delayed.time_us
		time_delay = max(single((imu.time_us - imu_sample_delayed.time_us)) * single(1e-6), dt_imu_avg);
		att_gain = single(0.5) * dt_imu_avg / time_delay;

		delta_angle_corr = delta_ang_error * att_gain;
		
		vel_gain = dt_ekf_avg / saturation(params.vel_Tau, dt_ekf_avg, single(10.0));
		pos_gain = dt_ekf_avg / saturation(params.pos_Tau, dt_ekf_avg, single(10.0));

		vel_err = states.vel - output_delayed.vel;
		pos_err = states.pos - output_delayed.pos;

		vel_err_integ = vel_err_integ + vel_err;
		vel_correction = vel_err * vel_gain + vel_err_integ * (vel_gain^2) * 0.1;

		
		pos_err_integ = pos_err_integ + pos_err;
		pos_correction = pos_err * pos_gain + pos_err_integ * (pos_gain^2) * 0.1;

		for i = 1:3
            output_buffer(i,1).vel = output_buffer(i,1).vel + vel_correction;
            output_buffer(i,1).pos = output_buffer(i,1).pos + pos_correction;
        end

        output_new = output_buffer(head_index,:);
        output_new.vel = output_new.vel + vel_imu_rel_body_ned;
        
    end
    

    output_last = output_new;

    % output plot
%     dq_out = dq';
%     delta_angle_corr_out = delta_angle_corr;

end