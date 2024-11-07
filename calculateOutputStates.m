function [output_quat_nominal,output_vel,output_pos] = calculateOutputStates(imu,states)
    
    CONSTANTS_ONE_G = single(9.80665);

    persistent quat_nominal;   
    persistent vel;
    persistent pos;
    persistent quat_buffer;
    persistent vel_buffer;
    persistent pos_buffer;
    persistent head_index;
    persistent tail_index;
    persistent delta_angle_corr;
    persistent vel_err_integ;
    persistent pos_err_integ;
    if isempty(quat_nominal)
        quat_nominal = single([1 0 0 0]);
    end
    if isempty(vel)
        vel = single([0 0 0]);
    end
    if isempty(pos)
        pos = single([0 0 0]);
    end
    if isempty(quat_buffer)
        quat_buffer = [quat_nominal;
                       quat_nominal;
                       quat_nominal;];
    end
    if isempty(vel_buffer)
        vel_buffer = [vel;vel;vel];
    end
    if isempty(pos_buffer)
        pos_buffer = [pos;pos;pos];
    end
    if isempty(head_index)
        head_index = uint8(1);
    end
    if isempty(tail_index)
        tail_index = uint8(1);
    end
    if isempty(delta_angle_corr)
        delta_angle_corr = single([0 0 0]);
    end
    if isempty(vel_err_integ)
        vel_err_integ = single([0 0 0]);
    end
    if isempty(pos_err_integ)
        pos_err_integ = single([0 0 0]);
    end
	dt_scale_correction = dt_imu_avg / dt_ekf_avg;

	delta_angle = imu.delta_ang - states.delta_ang_bias * dt_scale_correction + delta_angle_corr;

% 	time_us = imu.time_us;
	
	dq = Quaternion_from_AxisAngle(delta_angle);

	quat_nominal = quat_nominal * dq;

	quat_nominal = quat_normalize(quat_nominal);

	R_to_earth_now = Quat2Tbn(output_new.quat_nominal);

	delta_vel_body = imu.delta_vel - states.delta_vel_bias * dt_scale_correction;

	
	delta_vel_earth = R_to_earth_now * delta_vel_body;

	delta_vel_earth(2) = delta_vel_earth(2) + CONSTANTS_ONE_G * imu.delta_vel_dt;

% 	if (imu.delta_vel_dt > 1e-4) 
% 		_vel_deriv = delta_vel_earth * (1.0f / imu.delta_vel_dt);
%     end
    
	
	vel_last = vel;
	vel = vel + delta_vel_earth;

	delta_pos_NED = (vel + vel_last) * (imu.delta_vel_dt * 0.5);
	pos = pos + delta_pos_NED;
	
	if (imu.delta_ang_dt > 1e-4) 			
		ang_rate = imu.delta_ang  / imu.delta_ang_dt;		
		vel_imu_rel_body = cross(ang_rate, params.imu_pos_body);	
		vel_imu_rel_body_ned = R_to_earth_now * vel_imu_rel_body;			
    end

	
	if (err_updated) 
		quat_buffer(mod(head_index,3),:) = quat_nominal;
        vel_buffer(mod(head_index,3),:) = vel;
        pos_buffer(mod(head_index,3),:) = pos;
		
		quat_delayed = quat_buffer(tail_index,:);
		vel_delayed = vel_buffer(tail_index,:);
        pos_delayed = pos_buffer(tail_index,:);

        head_index = head_index + 1;
        tail_index = tail_index + 1;

		q_error = quat_normalize(quat_inverse(states.quat_nominal) * quat_delayed);
        
        if q_error(1)>=single(1)
            scalar = -2;
        else
            scalar = 2;
        end
		

		delta_ang_error = [scalar*q_error(2)  scalar*q_error(3)  scalar*q_error(4)];

		time_delay = max((imu.time_us - imu_sample_delayed.time_us) * 1e-6, dt_imu_avg);
		att_gain = single(0.5) * dt_imu_avg / time_delay;

		delta_angle_corr = delta_ang_error * att_gain;
		
		vel_gain = dt_ekf_avg / saturation(params.vel_Tau, dt_ekf_avg, single(10.0));
		pos_gain = dt_ekf_avg / saturation(params.pos_Tau, dt_ekf_avg, single(10.0));

		vel_err = states.vel - vel_delayed;
		pos_err = states.pos - pos_delayed;

		vel_err_integ = vel_err_integ + vel_err;
		vel_correction = vel_err * vel_gain + vel_err_integ * (vel_gain^2) * 0.1;

		
		pos_err_integ = pos_err_integ + pos_err;
		pos_correction = pos_err * pos_gain + pos_err_integ * (pos_gain^2) * 0.1;

		for i = 1:3
            vel_buffer(i,:) = vel_buffer(i,:) + vel_correction;
            pos_buffer(i,:) = pos_buffer(i,:) + pos_correction;
        end
        output_quat_nominal = quat_buffer(head_index,:);
        output_vel = vel_buffer(head_index,:) + vel_imu_rel_body_ned;
        output_pos = pos_buffer(head_index,:);
    end
end