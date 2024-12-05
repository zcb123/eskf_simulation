function predictState()
    global states  dt_ekf_avg R_to_earth ;
    global CONSTANTS_ONE_G;
    global params imu_sample_delayed;
    global accel_lpf_NE;
    
    delta_ang = imu_sample_delayed.delta_ang - states.delta_ang_bias;
    assignin("base","delta_ang",delta_ang);

    q1 = states.quat_nominal(1);
    q2 = states.quat_nominal(2);
    q3 = states.quat_nominal(3);
    q4 = states.quat_nominal(4);
    
    ang_norm = norm(delta_ang);

    cos_ang = cos(ang_norm*0.5);
    cos_ang_half = cos(ang_norm*0.25);
     
    if(ang_norm>1e-7)
        % 等价于quatMult(Q,[delta_ang])
        sin_ang = sin(ang_norm*0.5);
        gain = sin_ang/ang_norm;
        dq1 = q1 * cos_ang + (-delta_ang(1) * q2 - delta_ang(2) * q3 - delta_ang(3) * q4) * gain;
		dq2 = q2 * cos_ang + (delta_ang(1) * q1 - delta_ang(2) * q4 + delta_ang(3) * q3) * gain;
		dq3 = q3 * cos_ang + (delta_ang(1) * q4 + delta_ang(2) * q1 - delta_ang(3) * q2) * gain;
		dq4 = q4 * cos_ang + (-delta_ang(1) * q3 + delta_ang(2)  * q2 + delta_ang(3) * q1) * gain;
		%相当于旋转一半的角度
		 sin_ang_half = sin(ang_norm * 0.25);
		gain = sin_ang_half / ang_norm;
		ddq1 = q1 * cos_ang_half + (-delta_ang(1) * q2 - delta_ang(2) * q3 - delta_ang(3) * q4) * gain;
		ddq2 = q2 * cos_ang_half + (delta_ang(1) * q1 - delta_ang(2) * q4 + delta_ang(3) * q3) * gain;
		ddq3 = q3 * cos_ang_half + (delta_ang(1) * q4 + delta_ang(2) * q1 - delta_ang(3) * q2) * gain;
		ddq4 = q4 * cos_ang_half + (-delta_ang(1) * q3 + delta_ang(2) * q2 + delta_ang(3) * q1) * gain;
        dq_dt = quat_normalize([dq1 dq2 dq3 dq4]);
        dq_dt2 = quat_normalize([ddq1 ddq2 ddq3 ddq4]);
    else 
		gain = cos_ang;
		dq1 = q1 * cos_ang + (-delta_ang(1) * q2 - delta_ang(2) * q3 - delta_ang(3) * q4) * gain * 0.5;
		dq2 = q2 * cos_ang + (delta_ang(1) * q1 - delta_ang(2) * q4 + delta_ang(3) * q3) * gain * 0.5;
		dq3 = q3 * cos_ang + (delta_ang(1) * q4 + delta_ang(2) * q1 - delta_ang(3) * q2) * gain * 0.5;
		dq4 = q4 * cos_ang + (-delta_ang(1) * q3 + delta_ang(2) * q2 + delta_ang(3) * q1) * gain * 0.5;

		gain = cos_ang_half * 0.25;
		ddq1 = q1 * cos_ang_half + (-delta_ang(1) * q2 - delta_ang(2) * q3 - delta_ang(3) * q4) * gain;
		ddq2 = q2 * cos_ang_half + (delta_ang(1) * q1 - delta_ang(2) * q4 + delta_ang(3) * q3) * gain;
		ddq3 = q3 * cos_ang_half + (delta_ang(1) * q4 + delta_ang(2) * q1 - delta_ang(3) * q2) * gain ;
		ddq4 = q4 * cos_ang_half + (-delta_ang(1) * q3 + delta_ang(2) * q2 + delta_ang(3) * q1) * gain;

		dq_dt = quat_normalize([dq1 dq2 dq3 dq4]);
        dq_dt2 = quat_normalize([ddq1 ddq2 ddq3 ddq4]);
    end

    assignin("base","dq_dt",dq_dt);
	states.quat_nominal = dq_dt;        %四元数的更新预测

	R_to_earth = Quat2Tbn(states.quat_nominal);

	corrected_delta_vel = imu_sample_delayed.delta_vel  - states.delta_vel_bias;
	corrected_delta_vel_ef = R_to_earth * corrected_delta_vel;
    assignin("base","corrected_delta_vel",corrected_delta_vel);
	
	alpha = 1.0 - imu_sample_delayed.delta_vel_dt;
	accel_lpf_NE = accel_lpf_NE * alpha + corrected_delta_vel_ef(1:2,1);

	% RK4
	dR_dt_t = Quat2Tbn(dq_dt);
	dR_dt2_t = Quat2Tbn(dq_dt2);

	k1_v_dot = corrected_delta_vel_ef;
	k1_p_dot = states.vel;

	k1_v = states.vel + k1_v_dot * 0.5;
	k2_v_dot = dR_dt2_t * corrected_delta_vel;
	k2_p_dot = k1_v;

	k2_v = states.vel + k2_v_dot * 0.5;
	k3_v_dot  = k2_v_dot;
	k3_p_dot = k2_v;

	k3_v = states.vel + k3_v_dot;
	k4_v_dot = dR_dt_t * corrected_delta_vel;
	k4_p_dot = k3_v;

	states.vel = states.vel + (k1_v_dot + 2 * k2_v_dot + 2 * k3_v_dot + k4_v_dot) / 6;
	states.vel(2) = states.vel(2) + CONSTANTS_ONE_G * imu_sample_delayed.delta_vel_dt;

	states.pos = states.pos + (k1_p_dot + 2 * k2_p_dot + 2 * k3_p_dot + k4_p_dot) * imu_sample_delayed.delta_vel_dt / 6;
	states.pos(2) = states.pos(2) + 0.5 * CONSTANTS_ONE_G * imu_sample_delayed.delta_vel_dt * imu_sample_delayed.delta_vel_dt;

    %% 状态限幅
    states.quat_nominal = saturation(states.quat_nominal,-1,1);
    states.vel = saturation(states.vel,-1000,1000);
    states.pos = saturation(states.pos,-1e6,1e6);

    %%  这个量暂时不用
    input = 0.5*(imu_sample_delayed.delta_ang_dt + imu_sample_delayed.delta_vel_dt);
    filter_update_s = params.filter_update_interval_us*1e-6;
    input = saturation(input,0.5*filter_update_s,2*filter_update_s);
    dt_ekf_avg = 0.99*dt_ekf_avg+0.01*double(input);
end

