function predictCovariance()

    INHIBIT_ACC_BIAS = bitshift(1,2);
    BADACC_BIASPNOISE = single(4.9);
    BADGYROPNOISE = single(0.2);
    
    global states dt_ekf_avg P accel_bias_inhibit R_to_earth;
    global params control_status fault_status imu_sample_delayed;
    global ang_rate_magnitude_filt accel_magnitude_filt accel_vec_filt;
    
    % assign intermediate state variables
	q1 = states.quat_nominal(1);
	q2 = states.quat_nominal(2);
	q3 = states.quat_nominal(3);
	q4 = states.quat_nominal(4);

	%dax = imu_sample_delayed.delta_ang(0);
	%day = imu_sample_delayed.delta_ang(1);
	%daz = imu_sample_delayed.delta_ang(2);

	dvx = imu_sample_delayed.delta_vel(1);
	dvy = imu_sample_delayed.delta_vel(2);
	dvz = imu_sample_delayed.delta_vel(3);

	%dax_b = states.delta_ang_bias(0);
	%day_b = states.delta_ang_bias(1);
	%daz_b = states.delta_ang_bias(2);

	dvx_b = states.delta_vel_bias(1);
	dvy_b = states.delta_vel_bias(2);
	dvz_b = states.delta_vel_bias(3);
    
%     R_to_earth = Quat2Tbn([q1 q2 q3 q4]);
	% Use average update interval to reduce accumulated covariance prediction errors due to small single frame dt values
	dt = dt_ekf_avg;
	dt_inv = 1 / dt;

	% convert rate of change of rate gyro bias (rad/s**2) as specified by the parameter to an expected change in delta angle (rad) since the last update
	d_ang_bias_sig = dt * dt * saturation(params.gyro_bias_p_noise, 0, 1);      %gyro_bias_p_noise = 1e-3   d_ang_bias_sig = 0.008*0.008*1e-3

	% convert rate of change of accelerometer bias (m/s**3) as specified by the parameter to an expected change in delta velocity (m/s) since the last update
	d_vel_bias_sig = dt * dt * saturation(params.accel_bias_p_noise, 0, 1);     %accel_bias_p_noise = 3e-3  d_vel_bias_sig = 0.008*0.008*3e-3

	% inhibit learning of imu accel bias if the manoeuvre levels are too high to protect against the effect of sensor nonlinearities or bad accel data is detected
	% xy accel bias learning is also disabled on ground as those states are poorly observable when perpendicular to the gravity vector
    alpha = saturation((dt / params.acc_bias_learn_tc), 0, 1);      %acc_bias_learn_tc = 0.5
	beta = 1 - alpha;
	ang_rate_magnitude_filt = max(dt_inv*norm(imu_sample_delayed.delta_ang), beta*ang_rate_magnitude_filt);
	accel_magnitude_filt = max(dt_inv*norm(imu_sample_delayed.delta_vel), beta*accel_magnitude_filt);
	accel_vec_filt = alpha * dt_inv*imu_sample_delayed.delta_vel + beta*accel_vec_filt;

	is_manoeuvre_level_high = logical(ang_rate_magnitude_filt > params.acc_bias_learn_gyr_lim...
					     || accel_magnitude_filt > params.acc_bias_learn_acc_lim);

	do_inhibit_all_axes = logical(bitand(params.fusion_mode , INHIBIT_ACC_BIAS)...    %   params.fusion_mode = 1
					 || is_manoeuvre_level_high...                              %manoeuvre:操纵
					 || fault_status.flags.bad_acc_vertical);
    
    persistent prev_dvel_bias_var
    if isempty(prev_dvel_bias_var)
        prev_dvel_bias_var = zeros(3,3);
    end

	for stateIndex = 12 : 14 
		index = stateIndex - 11;

		do_inhibit_axis = do_inhibit_all_axes || imu_sample_delayed.delta_vel_clipping(index);

		if (do_inhibit_axis) 
			% store the bias state variances to be reinstated later
			if ~accel_bias_inhibit(index)
				prev_dvel_bias_var(index) = P(stateIndex, stateIndex);
				accel_bias_inhibit(index) = true;
			end

    	else 
			if (accel_bias_inhibit(index)) 
				% reinstate the bias state variances
				P(stateIndex, stateIndex) = prev_dvel_bias_var(index);
				accel_bias_inhibit(index) = false;
			end
		end
	end

	% Don't continue to grow the earth field variances if they are becoming too large or we are not doing 3-axis fusion as this can make the covariance matrix badly conditioned
	%mag_I_sig;
    k_mag_i_id = uint8(16);
	if (control_status.flags.mag_3D && (P(k_mag_i_id, k_mag_i_id) + P(k_mag_i_id+1, k_mag_i_id+1) + P(k_mag_i_id+2, k_mag_i_id+2)) < 0.1) 
		mag_I_sig = dt * saturation(params.mage_p_noise, 0, 1);

    else 
		mag_I_sig = 0;
	end

	% Don't continue to grow the body field variances if they is becoming too large or we are not doing 3-axis fusion as this can make the covariance matrix badly conditioned
	%mag_B_sig;
    k_mag_bias_id = uint8(19);
	if (control_status.flags.mag_3D && (P(k_mag_bias_id, k_mag_bias_id) + P(k_mag_bias_id+1, k_mag_bias_id+1) + P(k_mag_bias_id+2, k_mag_bias_id+2)) < 0.1) 
		mag_B_sig = dt * saturation(params.magb_p_noise, 0, 1);

    else 
		mag_B_sig = 0.0;
	end

	%wind_vel_sig;

	% Calculate low pass filtered height rate
    persistent height_rate_lpf
    if isempty(height_rate_lpf)
        height_rate_lpf = single(0);
    end
	alpha_height_rate_lpf = 0.1 * dt; % 10 seconds time constant
	height_rate_lpf = height_rate_lpf * (1 - alpha_height_rate_lpf) + states.vel(2) * alpha_height_rate_lpf;

	% Don't continue to grow wind velocity state variances if they are becoming too large or we are not using wind velocity states as this can make the covariance matrix badly conditioned
	k_wind_vel_id = uint8(22);
    if (control_status.flags.wind && (P(k_wind_vel_id,k_wind_vel_id) + P(k_wind_vel_id+1,k_wind_vel_id+1)) < (params.initial_wind_uncertainty^2)) 
		wind_vel_sig = dt * saturation(params.wind_vel_p_noise, 0, 1) * (1 + params.wind_vel_p_noise_scaler * abs(height_rate_lpf));

    else 
		wind_vel_sig = 0;
    end
	% compute noise variance for stationary processes
	% Construct the process noise variance diagonal for those states with a stationary process model
	% These are kinematic states and their error growth is controlled separately by the IMU noise variances
	% assign IMU noise variances
	% inputs to the system are 3 delta angles and 3 delta velocities
	gyro_noise = saturation(params.gyro_noise, 0, 1);
	daxVar = (dt * gyro_noise)^2;			%
	dayVar = daxVar;
	dazVar = daxVar;
	% Accelerometer Clipping
	% delta velocity X: increase process noise if sample contained any X axis clipping
	if imu_sample_delayed.delta_ang_clipping(1) 
		daxVar = sq(dt * BADGYROPNOISE);		% (0.008*0.2)^2 = 2.56e-6
	end

	% delta velocity Y: increase process noise if sample contained any Y axis clipping
	if imu_sample_delayed.delta_ang_clipping(2) 
		dayVar = sq(dt * BADGYROPNOISE);
	end

	% delta velocity Z: increase process noise if sample contained any Z axis clipping
	if imu_sample_delayed.delta_ang_clipping(3) 
		dazVar = sq(dt * BADGYROPNOISE);
	end

	accel_noise = saturation(params.accel_noise, 0, 1);

    
	if (fault_status.flags.bad_acc_vertical) 
		% Increase accelerometer process noise if bad accel data is detected. Measurement errors due to
		% vibration induced clipping commonly reach an equivalent 0.5g offset.
		accel_noise = BADACC_BIASPNOISE;
	end

	
	dvxVar = (dt * accel_noise)^2 ;
    dvyVar = (dt * accel_noise)^2;
    dvzVar = (dt * accel_noise)^2;

	% Accelerometer Clipping
	% delta velocity X: increase process noise if sample contained any X axis clipping
	if imu_sample_delayed.delta_vel_clipping(1) 
		dvxVar = sq(dt * BADACC_BIASPNOISE);		
	end

	% delta velocity Y: increase process noise if sample contained any Y axis clipping
	if imu_sample_delayed.delta_vel_clipping(2) 
		dvyVar = sq(dt * BADACC_BIASPNOISE);
	end

	% delta velocity Z: increase process noise if sample contained any Z axis clipping
	if imu_sample_delayed.delta_vel_clipping(3) 
		dvzVar = sq(dt * BADACC_BIASPNOISE);
	end

	% predict the covariance
	% equations generated using EKF/python/ekf_derivation/main.py
    
	% Equations for covariance matrix prediction, without process noise~
	PS0 = 2*q3*q3;
	PS1 = 2*q4*q4 - 1;
	PS2 = PS0 + PS1;
	PS3 = q1*q3;
	PS4 = q2*q4;
	PS5 = 2*PS3 + 2*PS4;
	PS6 = q1*q4;
	PS7 = q2*q3;
	PS8 = 2*PS6 - 2*PS7;
	PS9 = PS2*P(10,10) - PS5*P(10,12) + PS8*P(10,11) + P(1,10);
	PS10 = PS2*P(10,12) - PS5*P(12,12) + PS8*P(11,12) + P(1,12);
	PS11 = PS2*P(10,11) - PS5*P(11,12) + PS8*P(11,11) + P(1,11);
	PS12 = PS2*P(1,10) - PS5*P(1,12) + PS8*P(1,11) + P(1,1);
	PS13 = 2*q2*q2;
	PS14 = PS1 + PS13;
	PS15 = PS6 + PS7;
	PS16 = 2*PS9;
	PS17 = q1*q2;
	PS18 = q3*q4;
	PS19 = 2*PS17 - 2*PS18;
	PS20 = PS2*P(2,10) - PS5*P(2,12) + PS8*P(2,11) + P(1,2);
	PS21 = PS0 + PS13 - 1;
	PS22 = 2*PS17 + 2*PS18;
	PS23 = PS3 - PS4;
	PS24 = PS2*P(3,10) - PS5*P(3,12) + PS8*P(3,11) + P(1,3);
	PS25 = PS2*P(10,13);
	PS26 = PS25 - PS5*P(12,13) + PS8*P(11,13) + P(1,13);
	PS27 = dvy - dvy_b; %dVel_y - dv_by;
	PS28 = PS22*PS27;
	PS29 = dvx - dvx_b; %dVel_x - dv_bx;
	PS30 = 2*PS29;
	PS31 = PS23*PS30;
	PS32 = dvz - dvz_b; %dVel_z - dv_bz;
	PS33 = PS21*PS32;
	PS34 = -PS28 + PS31 + PS33;
	PS35 = PS15*PS30;
	PS36 = PS19*PS32;
	PS37 = PS14*PS27;
	PS38 = PS35 - PS36 - PS37;
	PS39 = -PS5*P(12,15);
	PS40 = PS2*P(10,15) + PS39 + PS8*P(11,15) + P(1,15);
	PS41 = PS8*P(11,14);
	PS42 = PS2*P(10,14) + PS41 - PS5*P(12,14) + P(1,14);
	PS43 = PS2*P(4,10) - PS5*P(4,12) + PS8*P(4,11) + P(1,4);
	PS44 = PS32*PS5;
	PS45 = PS27*PS8;
	PS46 = PS2*PS29;
	PS47 = -PS44 + PS45 + PS46;
	PS48 = PS28 - PS31 - PS33;
	PS49 = 2*PS26;
	PS50 = PS2*P(5,10) - PS5*P(5,12) + PS8*P(5,11) + P(1,5);
	PS51 = -PS35 + PS36 + PS37;
	PS52 = PS44 - PS45 - PS46;
	PS53 = PS2*P(6,10) - PS5*P(6,12) + PS8*P(6,11) + P(1,6);
	PS54 = 2*PS15;
	PS55 = PS14*P(11,11) + PS19*P(11,12) - PS54*P(10,11) + P(2,11);
	PS56 = PS14*P(10,11) + PS19*P(10,12) - PS54*P(10,10) + P(2,10);
	PS57 = PS14*P(11,12) + PS19*P(12,12) - PS54*P(10,12) + P(2,12);
	PS58 = PS14*P(2,11) + PS19*P(2,12) - PS54*P(2,10) + P(2,2);
	PS59 = 2*PS23;
	PS60 = PS14*P(3,11) + PS19*P(3,12) - PS54*P(3,10) + P(2,3);
	PS61 = -PS54*P(10,13);
	PS62 = PS14*P(11,13) + PS19*P(12,13) + PS61 + P(2,13);
	PS63 = PS19*P(12,15);
	PS64 = PS14*P(11,15) - PS54*P(10,15) + PS63 + P(2,15);
	PS65 = PS14*P(11,14);
	PS66 = PS19*P(12,14) - PS54*P(10,14) + PS65 + P(2,14);
	PS67 = PS14*P(4,11) + PS19*P(4,12) - PS54*P(4,10) + P(2,4);
	PS68 = PS14*P(1,11) + PS19*P(1,12) - PS54*P(1,10) + P(1,2);
	PS69 = PS14*P(5,11) + PS19*P(5,12) - PS54*P(5,10) + P(2,5);
	PS70 = PS14*P(6,11) + PS19*P(6,12) - PS54*P(6,10) + P(2,6);
	PS71 = PS21*P(12,12) - PS22*P(11,12) + PS59*P(10,12) + P(3,12);
	PS72 = PS21*P(11,12) - PS22*P(11,11) + PS59*P(10,11) + P(3,11);
	PS73 = PS21*P(10,12) - PS22*P(10,11) + PS59*P(10,10) + P(3,10);
	PS74 = PS21*P(3,12) - PS22*P(3,11) + PS59*P(3,10) + P(3,3);
	PS75 = PS59*P(10,13);
	PS76 = PS21*P(12,13) - PS22*P(11,13) + PS75 + P(3,13);
	PS77 = PS21*P(2,12) - PS22*P(2,11) + PS59*P(2,10) + P(2,3);
	PS78 = PS21*P(12,15);
	PS79 = -PS22*P(11,15) + PS59*P(10,15) + PS78 + P(3,15);
	PS80 = -PS22*P(11,14);
	PS81 = PS21*P(12,14) + PS59*P(10,14) + PS80 + P(3,14);
	PS82 = PS21*P(4,12) - PS22*P(4,11) + PS59*P(4,10) + P(3,4);
	PS83 = PS21*P(1,12) - PS22*P(1,11) + PS59*P(1,10) + P(1,3);
	PS84 = PS21*P(5,12) - PS22*P(5,11) + PS59*P(5,10) + P(3,5);
	PS85 = PS21*P(6,12) - PS22*P(6,11) + PS59*P(6,10) + P(3,6);
	PS86 = PS2*P(13,13) - PS34*P(2,13) - PS38*P(3,13) - PS5*P(13,15) + PS8*P(13,14) + P(4,13);
	PS87 = PS2*P(2,13) - PS34*P(2,2) - PS38*P(2,3) - PS5*P(2,15) + PS8*P(2,14) + P(2,4);
	PS88 = PS2*P(3,13) - PS34*P(2,3) - PS38*P(3,3) - PS5*P(3,15) + PS8*P(3,14) + P(3,4);
	PS89 = PS2*P(13,15) - PS34*P(2,15) - PS38*P(3,15) - PS5*P(15,15) + PS8*P(14,15) + P(4,15);
	PS90 = PS2*P(13,14) - PS34*P(2,14) - PS38*P(3,14) - PS5*P(14,15) + PS8*P(14,14) + P(4,14);
	PS91 = PS2*P(4,13) - PS34*P(2,4) - PS38*P(3,4) - PS5*P(4,15) + PS8*P(4,14) + P(4,4);
	PS92 = PS2*P(1,13) - PS34*P(1,2) - PS38*P(1,3) - PS5*P(1,15) + PS8*P(1,14) + P(1,4);
	PS93 = PS2*P(5,13) - PS34*P(2,5) - PS38*P(3,5) - PS5*P(5,15) + PS8*P(5,14) + P(4,5);
	PS94 = PS2*P(6,13) - PS34*P(2,6) - PS38*P(3,6) - PS5*P(6,15) + PS8*P(6,14) + P(4,6);
	PS95 = PS14*P(14,14) + PS19*P(14,15) - PS47*P(3,14) - PS48*P(1,14) - PS54*P(13,14) + P(5,14);
	PS96 = PS14*P(1,14) + PS19*P(1,15) - PS47*P(1,3) - PS48*P(1,1) - PS54*P(1,13) + P(1,5);
	PS97 = PS14*P(13,14) + PS19*P(13,15) - PS47*P(3,13) - PS48*P(1,13) - PS54*P(13,13) + P(5,13);
	PS98 = PS14*P(14,15) + PS19*P(15,15) - PS47*P(3,15) - PS48*P(1,15) - PS54*P(13,15) + P(5,15);
	PS99 = PS14*P(5,14) + PS19*P(5,15) - PS47*P(3,5) - PS48*P(1,5) - PS54*P(5,13) + P(5,5);
	PS100 = PS14*P(6,14) + PS19*P(6,15) - PS47*P(3,6) - PS48*P(1,6) - PS54*P(6,13) + P(5,6);
	PS101 = PS21*P(15,15) - PS22*P(14,15) - PS51*P(1,15) - PS52*P(2,15) + PS59*P(13,15) + P(6,15);
	PS102 = PS21*P(14,15) - PS22*P(14,14) - PS51*P(1,14) - PS52*P(2,14) + PS59*P(13,14) + P(6,14);
	PS103 = PS21*P(13,15) - PS22*P(13,14) - PS51*P(1,13) - PS52*P(2,13) + PS59*P(13,13) + P(6,13);
	PS104 = PS21*P(6,15) - PS22*P(6,14) - PS51*P(1,6) - PS52*P(2,6) + PS59*P(6,13) + P(6,6);

	% covariance update
	nextP = zeros(23,23);

	% calculate variances and upper diagonal covariances for quaternion, velocity, position and gyro bias states
	% 计算四元数、速度、位置、陀螺仪零偏状态的上三角对角阵
	nextP(1,1) = -PS10*PS5 + PS11*PS8 + PS12 + PS2*PS9;
	nextP(1,2) = PS10*PS19 + PS11*PS14 - PS15*PS16 + PS20;
	nextP(2,2) = PS14*PS55 + PS19*PS57 - PS54*PS56 + PS58;
	nextP(1,3) = PS10*PS21 - PS11*PS22 + PS16*PS23 + PS24;
	nextP(2,3) = PS21*PS57 - PS22*PS55 + PS56*PS59 + PS60;
	nextP(3,3) = PS21*PS71 - PS22*PS72 + PS59*PS73 + PS74;
	nextP(1,4) = PS2*PS26 - PS20*PS34 - PS24*PS38 - PS40*PS5 + PS42*PS8 + PS43;
	nextP(2,4) = PS2*PS62 - PS34*PS58 - PS38*PS60 - PS5*PS64 + PS66*PS8 + PS67;
	nextP(3,4) = PS2*PS76 - PS34*PS77 - PS38*PS74 - PS5*PS79 + PS8*PS81 + PS82;
	nextP(4,4) = PS2*PS86 - PS34*PS87 - PS38*PS88 - PS5*PS89 + PS8*PS90 + PS91;
	nextP(1,5) = -PS12*PS48 + PS14*PS42 - PS15*PS49 + PS19*PS40 - PS24*PS47 + PS50;
	nextP(2,5) = PS14*PS66 + PS19*PS64 - PS47*PS60 - PS48*PS68 - PS54*PS62 + PS69;
	nextP(3,5) = PS14*PS81 + PS19*PS79 - PS47*PS74 - PS48*PS83 - PS54*PS76 + PS84;
	nextP(4,5) = PS14*PS90 + PS19*PS89 - PS47*PS88 - PS48*PS92 - PS54*PS86 + PS93;
	nextP(5,5) = PS14*PS95 + PS19*PS98 - PS47*(PS14*P(3,14) + PS19*P(3,15) - PS47*P(3,3) - PS48*P(1,3) - PS54*P(3,13) + P(3,5)) - PS48*PS96 - PS54*PS97 + PS99;
	nextP(1,6) = -PS12*PS51 - PS20*PS52 + PS21*PS40 - PS22*PS42 + PS23*PS49 + PS53;
	nextP(2,6) = PS21*PS64 - PS22*PS66 - PS51*PS68 - PS52*PS58 + PS59*PS62 + PS70;
	nextP(3,6) = PS21*PS79 - PS22*PS81 - PS51*PS83 - PS52*PS77 + PS59*PS76 + PS85;
	nextP(4,6) = PS21*PS89 - PS22*PS90 - PS51*PS92 - PS52*PS87 + PS59*PS86 + PS94;
	nextP(5,6) = PS100 + PS21*PS98 - PS22*PS95 - PS51*PS96 - PS52*(PS14*P(2,14) + PS19*P(2,15) - PS47*P(2,3) - PS48*P(1,2) - PS54*P(2,13) + P(2,5)) + PS59*PS97;
	nextP(6,6) = PS101*PS21 - PS102*PS22 + PS103*PS59 + PS104 - PS51*(PS21*P(1,15) - PS22*P(1,14) - PS51*P(1,1) - PS52*P(1,2) + PS59*P(1,13) + P(1,6)) - PS52*(PS21*P(2,15) - PS22*P(2,14) - PS51*P(1,2) - PS52*P(2,2) + PS59*P(2,13) + P(2,6));
	nextP(1,7) = PS2*P(7,10) + PS43*dt - PS5*P(7,12) + PS8*P(7,11) + P(1,7);
	nextP(2,7) = PS14*P(7,11) + PS19*P(7,12) - PS54*P(7,10) + PS67*dt + P(2,7);
	nextP(3,7) = PS21*P(7,12) - PS22*P(7,11) + PS59*P(7,10) + PS82*dt + P(3,7);
	nextP(4,7) = PS2*P(7,13) - PS34*P(2,7) - PS38*P(3,7) - PS5*P(7,15) + PS8*P(7,14) + PS91*dt + P(4,7);
	nextP(5,7) = PS14*P(7,14) + PS19*P(7,15) - PS47*P(3,7) - PS48*P(1,7) - PS54*P(7,13) + P(5,7) + dt*(PS14*P(4,14) + PS19*P(4,15) - PS47*P(3,4) - PS48*P(1,4) - PS54*P(4,13) + P(4,5));
	nextP(6,7) = PS21*P(7,15) - PS22*P(7,14) - PS51*P(1,7) - PS52*P(2,7) + PS59*P(7,13) + P(6,7) + dt*(PS21*P(4,15) - PS22*P(4,14) - PS51*P(1,4) - PS52*P(2,4) + PS59*P(4,13) + P(4,6));
	nextP(7,7) = P(4,7)*dt + P(7,7) + dt*(P(4,4)*dt + P(4,7));
	nextP(1,8) = PS2*P(8,10) - PS5*P(8,12) + PS50*dt + PS8*P(8,11) + P(1,8);
	nextP(2,8) = PS14*P(8,11) + PS19*P(8,12) - PS54*P(8,10) + PS69*dt + P(2,8);
	nextP(3,8) = PS21*P(8,12) - PS22*P(8,11) + PS59*P(8,10) + PS84*dt + P(3,8);
	nextP(4,8) = PS2*P(8,13) - PS34*P(2,8) - PS38*P(3,8) - PS5*P(8,15) + PS8*P(8,14) + PS93*dt + P(4,8);
	nextP(5,8) = PS14*P(8,14) + PS19*P(8,15) - PS47*P(3,8) - PS48*P(1,8) - PS54*P(8,13) + PS99*dt + P(5,8);
	nextP(6,8) = PS21*P(8,15) - PS22*P(8,14) - PS51*P(1,8) - PS52*P(2,8) + PS59*P(8,13) + P(6,8) + dt*(PS21*P(5,15) - PS22*P(5,14) - PS51*P(1,5) - PS52*P(2,5) + PS59*P(5,13) + P(5,6));
	nextP(7,8) = P(4,8)*dt + P(7,8) + dt*(P(4,5)*dt + P(5,7));
	nextP(8,8) = P(5,8)*dt + P(8,8) + dt*(P(5,5)*dt + P(5,8));
	nextP(1,9) = PS2*P(9,10) - PS5*P(9,12) + PS53*dt + PS8*P(9,11) + P(1,9);
	nextP(2,9) = PS14*P(9,11) + PS19*P(9,12) - PS54*P(9,10) + PS70*dt + P(2,9);
	nextP(3,9) = PS21*P(9,12) - PS22*P(9,11) + PS59*P(9,10) + PS85*dt + P(3,9);
	nextP(4,9) = PS2*P(9,13) - PS34*P(2,9) - PS38*P(3,9) - PS5*P(9,15) + PS8*P(9,14) + PS94*dt + P(4,9);
	nextP(5,9) = PS100*dt + PS14*P(9,14) + PS19*P(9,15) - PS47*P(3,9) - PS48*P(1,9) - PS54*P(9,13) + P(5,9);
	nextP(6,9) = PS104*dt + PS21*P(9,15) - PS22*P(9,14) - PS51*P(1,9) - PS52*P(2,9) + PS59*P(9,13) + P(6,9);
	nextP(7,9) = P(4,9)*dt + P(7,9) + dt*(P(4,6)*dt + P(6,7));
	nextP(8,9) = P(5,9)*dt + P(8,9) + dt*(P(5,6)*dt + P(6,8));
	nextP(9,9) = P(6,9)*dt + P(9,9) + dt*(P(6,6)*dt + P(6,9));
	nextP(1,10) = PS9;
	nextP(2,10) = PS56;
	nextP(3,10) = PS73;
	nextP(4,10) = PS25 - PS34*P(2,10) - PS38*P(3,10) - PS5*P(10,15) + PS8*P(10,14) + P(4,10);
	nextP(5,10) = PS14*P(10,14) + PS19*P(10,15) - PS47*P(3,10) - PS48*P(1,10) + PS61 + P(5,10);
	nextP(6,10) = PS21*P(10,15) - PS22*P(10,14) - PS51*P(1,10) - PS52*P(2,10) + PS75 + P(6,10);
	nextP(7,10) = P(4,10)*dt + P(7,10);
	nextP(8,10) = P(5,10)*dt + P(8,10);
	nextP(9,10) = P(6,10)*dt + P(9,10);
	nextP(10,10) = P(10,10);
	nextP(1,11) = PS11;
	nextP(2,11) = PS55;
	nextP(3,11) = PS72;
	nextP(4,11) = PS2*P(11,13) - PS34*P(2,11) - PS38*P(3,11) + PS41 - PS5*P(11,15) + P(4,11);
	nextP(5,11) = PS19*P(11,15) - PS47*P(3,11) - PS48*P(1,11) - PS54*P(11,13) + PS65 + P(5,11);
	nextP(6,11) = PS21*P(11,15) - PS51*P(1,11) - PS52*P(2,11) + PS59*P(11,13) + PS80 + P(6,11);
	nextP(7,11) = P(4,11)*dt + P(7,11);
	nextP(8,11) = P(5,11)*dt + P(8,11);
	nextP(9,11) = P(6,11)*dt + P(9,11);
	nextP(10,11) = P(10,11);
	nextP(11,11) = P(11,11);
	nextP(1,12) = PS10;
	nextP(2,12) = PS57;
	nextP(3,12) = PS71;
	nextP(4,12) = PS2*P(12,13) - PS34*P(2,12) - PS38*P(3,12) + PS39 + PS8*P(12,14) + P(4,12);
	nextP(5,12) = PS14*P(12,14) - PS47*P(3,12) - PS48*P(1,12) - PS54*P(12,13) + PS63 + P(5,12);
	nextP(6,12) = -PS22*P(12,14) - PS51*P(1,12) - PS52*P(2,12) + PS59*P(12,13) + PS78 + P(6,12);
	nextP(7,12) = P(4,12)*dt + P(7,12);
	nextP(8,12) = P(5,12)*dt + P(8,12);
	nextP(9,12) = P(6,12)*dt + P(9,12);
	nextP(10,12) = P(10,12);
	nextP(11,12) = P(11,12);
	nextP(12,12) = P(12,12);

    
	 % delta angle noise
	 % R * diag(nAng_x*nAng_x, nAng_y*nAng_y, nAng_z*nAng_z) * R'
	
	mR00 = R_to_earth(1,1); mR01 = R_to_earth(1,2); mR02 = R_to_earth(1,3);
	mR10 = R_to_earth(2,1); mR11 = R_to_earth(2,2); mR12 = R_to_earth(2,3);
	mR20 = R_to_earth(3,1); mR21 = R_to_earth(3,2); mR22 = R_to_earth(3,3);

        r00_sx = mR00 * daxVar;
        r01_sy = mR01 * dayVar;
        r02_sz = mR02 * dazVar;
        r10_sx = mR10 * daxVar;
        r11_sy = mR11 * dayVar;
        r12_sz = mR12 * dazVar;
	r20_sx = mR20 * daxVar;
	r21_sy = mR21 * dayVar;
	r22_sz = mR22 * dazVar;
        r00_sx_r10_plus_r01_sy_r11_plus_r02_sz_r12 = r00_sx * mR10 + r01_sy * mR11 + r02_sz * mR12;
        r00_sx_r20_plus_r01_sy_r21_plus_r02_sz_r22 = r00_sx * mR20 + r01_sy * mR21 + r02_sz * mR22;
        r10_r20_sx_plus_r11_r21_sy_plus_r12_r22_sz = r10_sx * mR20 + r11_sy * mR21 + r12_sz * mR22;
    global delta_angle_var_accum;
    
	[nextP(1,1),delta_angle_var_accum(1)] = kahanSummation(nextP(1,1), mR00 * r00_sx + mR01 * r01_sy + mR02 * r02_sz, delta_angle_var_accum(1));
	nextP(1,2) = nextP(1,2) + r00_sx_r10_plus_r01_sy_r11_plus_r02_sz_r12;
	nextP(1,3) = nextP(1,3) + r00_sx_r20_plus_r01_sy_r21_plus_r02_sz_r22;
	[nextP(2,2),delta_angle_var_accum(2)] = kahanSummation(nextP(2,2), mR10 * r10_sx + mR11 * r11_sy + mR12 * r12_sz, delta_angle_var_accum(2));
	nextP(2,3) = nextP(2,3) + r10_r20_sx_plus_r11_r21_sy_plus_r12_r22_sz;
	[nextP(3,3),delta_angle_var_accum(3)] = kahanSummation(nextP(3,3), mR20 * r20_sx + mR21 * r21_sy + mR22 * r22_sz, delta_angle_var_accum(3));
    
    
	
	 % delta velocity noise
	 % R * diag(nVel_x, nVel_y, nVel_z) * R'
	
        r00_sx = mR00 * dvxVar;
        r01_sy = mR01 * dvyVar;
        r02_sz = mR02 * dvzVar;
        r10_sx = mR10 * dvxVar;
        r11_sy = mR11 * dvyVar;
        r12_sz = mR12 * dvzVar;
	r20_sx = mR20 * dvxVar;
	r21_sy = mR21 * dvyVar;
	r22_sz = mR22 * dvzVar;
        r00_sx_r10_plus_r01_sy_r11_plus_r02_sz_r12 = r00_sx * mR10 + r01_sy * mR11 + r02_sz * mR12;
        r00_sx_r20_plus_r01_sy_r21_plus_r02_sz_r22 = r00_sx * mR20 + r01_sy * mR21 + r02_sz * mR22;
        r10_r20_sx_plus_r11_r21_sy_plus_r12_r22_sz = r10_sx * mR20 + r11_sy * mR21 + r12_sz * mR22;
    
    global delta_vel_var_accum;
    
	[nextP(4,4),delta_vel_var_accum(1)] = kahanSummation(nextP(4,4), mR00*r00_sx + mR01*r01_sy + mR02*r02_sz, delta_vel_var_accum(1));
	nextP(4,5) = nextP(4,5) + r00_sx_r10_plus_r01_sy_r11_plus_r02_sz_r12;
	nextP(4,6) = nextP(4,6) + r00_sx_r20_plus_r01_sy_r21_plus_r02_sz_r22;
	[nextP(5,5),delta_vel_var_accum(2)] = kahanSummation(nextP(5,5), mR10*r10_sx + mR11*r11_sy + mR12*r12_sz, delta_vel_var_accum(2));
	nextP(5,6) = nextP(5,6) + r10_r20_sx_plus_r11_r21_sy_plus_r12_r22_sz;
	[nextP(6,6),delta_vel_var_accum(3)] = kahanSummation(nextP(6,6), mR20*r20_sx + mR21 * r21_sy + mR22 * r22_sz, delta_vel_var_accum(3));

	% process noise contribution for delta angle states can be very small compared to
	% the variances, therefore use algorithm to minimise numerical error

	noise_delta_ang_bias = (d_ang_bias_sig)^2;
    global delta_angle_bias_var_accum;
   
	for i = 10:12 
		index = i - 9;
		[nextP(i, i),delta_angle_bias_var_accum(index)] = kahanSummation(nextP(i, i), noise_delta_ang_bias, delta_angle_bias_var_accum(index));
    end

    global delta_vel_bias_var_accum;
	noise_delta_vel_bias = (d_vel_bias_sig)^2;
	if ~accel_bias_inhibit(1) 
		% calculate variances and upper diagonal covariances for IMU X axis delta velocity bias state
		nextP(1,13) = PS26;
		nextP(2,13) = PS62;
		nextP(3,13) = PS76;
		nextP(4,13) = PS86;
		nextP(5,13) = PS97;
		nextP(6,13) = PS103;
		nextP(7,13) = P(4,13)*dt + P(7,13);
		nextP(8,13) = P(5,13)*dt + P(8,13);
		nextP(9,13) = P(6,13)*dt + P(9,13);
		nextP(10,13) = P(10,13);
		nextP(11,13) = P(11,13);
		nextP(12,13) = P(12,13);
		nextP(13,13) = P(13,13);

		% add process noise that is not from the IMU
		% process noise contribution for delta velocity states can be very small compared to
		% the variances, therefore use algorithm to minimise numerical error

		[nextP(13, 13),delta_vel_bias_var_accum(1)] = kahanSummation(nextP(13, 13), noise_delta_vel_bias, delta_vel_bias_var_accum(1));
        %disp('nexp(13,13)');
	else 
		%nextP.uncorrelateCovarianceSetVariance<1>(13, prev_dvel_bias_var(1));
        nextP(1:23,13) = 0;
        nextP(13,1:23) = 0;
        nextP(13,13) = prev_dvel_bias_var(1);
		delta_vel_bias_var_accum(1) = 0;
    
	end

	if ~accel_bias_inhibit(2) 
		% calculate variances and upper diagonal covariances for IMU Y axis delta velocity bias state
		nextP(1,14) = PS42;
		nextP(2,14) = PS66;
		nextP(3,14) = PS81;
		nextP(4,14) = PS90;
		nextP(5,14) = PS95;
		nextP(6,14) = PS102;
		nextP(7,14) = P(4,14)*dt + P(7,14);
		nextP(8,14) = P(5,14)*dt + P(8,14);
		nextP(9,14) = P(6,14)*dt + P(9,14);
		nextP(10,14) = P(10,14);
		nextP(11,14) = P(11,14);
		nextP(12,14) = P(12,14);
		nextP(13,14) = P(13,14);
		nextP(14,14) = P(14,14);

		% add process noise that is not from the IMU
		% process noise contribution for delta velocity states can be very small compared to
		% the variances, therefore use algorithm to minimise numerical error
		[nextP(14, 14),delta_vel_bias_var_accum(2)] = kahanSummation(nextP(14, 14), noise_delta_vel_bias, delta_vel_bias_var_accum(2));
        %disp('nexp(14,14)');
	else 
		%nextP.uncorrelateCovarianceSetVariance<1>(14, prev_dvel_bias_var(2));
        nextP(1:23,14) = 0;
        nextP(14,1:23) = 0;
        nextP(14,14) = prev_dvel_bias_var(2);
		delta_vel_bias_var_accum(2) = 0;

	end

	if ~accel_bias_inhibit(3) 
		% calculate variances and upper diagonal covariances for IMU Z axis delta velocity bias state
		nextP(1,15) = PS40;
		nextP(2,15) = PS64;
		nextP(3,15) = PS79;
		nextP(4,15) = PS89;
		nextP(5,15) = PS98;
		nextP(6,15) = PS101;
		nextP(7,15) = P(4,15)*dt + P(7,15);
		nextP(8,15) = P(5,15)*dt + P(8,15);
		nextP(9,15) = P(6,15)*dt + P(9,15);
		nextP(10,15) = P(10,15);
		nextP(11,15) = P(11,15);
		nextP(12,15) = P(12,15);
		nextP(13,15) = P(13,15);
		nextP(14,15) = P(14,15);
		nextP(15,15) = P(15,15);
		% add process noise that is not from the IMU
		% process noise contribution for delta velocity states can be very small compared to
		% the variances, therefore use algorithm to minimise numerical error
		nextP(15, 15) = kahanSummation(nextP(15, 15), noise_delta_vel_bias, delta_vel_bias_var_accum(3));
        %disp('nexp(15,15)');
	else 
		%nextP.uncorrelateCovarianceSetVariance<2>(15, prev_dvel_bias_var(3));
        nextP(1:23,15) = 0;
        nextP(15,1:23) = 0;
        nextP(15,15) = prev_dvel_bias_var(3);
		delta_vel_bias_var_accum(3) = 0;
	end

	% Don't do covariance prediction on magnetic field states unless we are using 4-axis fusion
	if (control_status.flags.mag_3D) 
		% calculate variances and upper diagonal covariances for earth and body magnetic field states

		nextP(1,16) = PS2*P(10,16) - PS5*P(12,16) + PS8*P(11,16) + P(1,16);
		nextP(2,16) = PS14*P(11,16) + PS19*P(12,16) - PS54*P(10,16) + P(2,16);
		nextP(3,16) = PS21*P(12,16) - PS22*P(11,16) + PS59*P(10,16) + P(3,16);
		nextP(4,16) = PS2*P(13,16) - PS34*P(2,16) - PS38*P(3,16) - PS5*P(15,16) + PS8*P(14,16) + P(4,16);
		nextP(5,16) = PS14*P(14,16) + PS19*P(15,16) - PS47*P(3,16) - PS48*P(1,16) - PS54*P(13,16) + P(5,16);
		nextP(6,16) = PS21*P(15,16) - PS22*P(14,16) - PS51*P(1,16) - PS52*P(2,16) + PS59*P(13,16) + P(6,16);
		nextP(7,16) = P(4,16)*dt + P(7,16);
		nextP(8,16) = P(5,16)*dt + P(8,16);
		nextP(9,16) = P(6,16)*dt + P(9,16);
		nextP(10,16) = P(10,16);
		nextP(11,16) = P(11,16);
		nextP(12,16) = P(12,16);
		nextP(13,16) = P(13,16);
		nextP(14,16) = P(14,16);
		nextP(15,16) = P(15,16);
		nextP(16,16) = P(16,16);
		nextP(1,17) = PS2*P(10,17) - PS5*P(12,17) + PS8*P(11,17) + P(1,17);
		nextP(2,17) = PS14*P(11,17) + PS19*P(12,17) - PS54*P(10,17) + P(2,17);
		nextP(3,17) = PS21*P(12,17) - PS22*P(11,17) + PS59*P(10,17) + P(3,17);
		nextP(4,17) = PS2*P(13,17) - PS34*P(2,17) - PS38*P(3,17) - PS5*P(15,17) + PS8*P(14,17) + P(4,17);
		nextP(5,17) = PS14*P(14,17) + PS19*P(15,17) - PS47*P(3,17) - PS48*P(1,17) - PS54*P(13,17) + P(5,17);
		nextP(6,17) = PS21*P(15,17) - PS22*P(14,17) - PS51*P(1,17) - PS52*P(2,17) + PS59*P(13,17) + P(6,17);
		nextP(7,17) = P(4,17)*dt + P(7,17);
		nextP(8,17) = P(5,17)*dt + P(8,17);
		nextP(9,17) = P(6,17)*dt + P(9,17);
		nextP(10,17) = P(10,17);
		nextP(11,17) = P(11,17);
		nextP(12,17) = P(12,17);
		nextP(13,17) = P(13,17);
		nextP(14,17) = P(14,17);
		nextP(15,17) = P(15,17);
		nextP(16,17) = P(16,17);
		nextP(17,17) = P(17,17);
		nextP(1,18) = PS2*P(10,18) - PS5*P(12,18) + PS8*P(11,18) + P(1,18);
		nextP(2,18) = PS14*P(11,18) + PS19*P(12,18) - PS54*P(10,18) + P(2,18);
		nextP(3,18) = PS21*P(12,18) - PS22*P(11,18) + PS59*P(10,18) + P(3,18);
		nextP(4,18) = PS2*P(13,18) - PS34*P(2,18) - PS38*P(3,18) - PS5*P(15,18) + PS8*P(14,18) + P(4,18);
		nextP(5,18) = PS14*P(14,18) + PS19*P(15,18) - PS47*P(3,18) - PS48*P(1,18) - PS54*P(13,18) + P(5,18);
		nextP(6,18) = PS21*P(15,18) - PS22*P(14,18) - PS51*P(1,18) - PS52*P(2,18) + PS59*P(13,18) + P(6,18);
		nextP(7,18) = P(4,18)*dt + P(7,18);
		nextP(8,18) = P(5,18)*dt + P(8,18);
		nextP(9,18) = P(6,18)*dt + P(9,18);
		nextP(10,18) = P(10,18);
		nextP(11,18) = P(11,18);
		nextP(12,18) = P(12,18);
		nextP(13,18) = P(13,18);
		nextP(14,18) = P(14,18);
		nextP(15,18) = P(15,18);
		nextP(16,18) = P(16,18);
		nextP(17,18) = P(17,18);
		nextP(18,18) = P(18,18);
		nextP(1,19) = PS2*P(10,19) - PS5*P(12,19) + PS8*P(11,19) + P(1,19);
		nextP(2,19) = PS14*P(11,19) + PS19*P(12,19) - PS54*P(10,19) + P(2,19);
		nextP(3,19) = PS21*P(12,19) - PS22*P(11,19) + PS59*P(10,19) + P(3,19);
		nextP(4,19) = PS2*P(13,19) - PS34*P(2,19) - PS38*P(3,19) - PS5*P(15,19) + PS8*P(14,19) + P(4,19);
		nextP(5,19) = PS14*P(14,19) + PS19*P(15,19) - PS47*P(3,19) - PS48*P(1,19) - PS54*P(13,19) + P(5,19);
		nextP(6,19) = PS21*P(15,19) - PS22*P(14,19) - PS51*P(1,19) - PS52*P(2,19) + PS59*P(13,19) + P(6,19);
		nextP(7,19) = P(4,19)*dt + P(7,19);
		nextP(8,19) = P(5,19)*dt + P(8,19);
		nextP(9,19) = P(6,19)*dt + P(9,19);
		nextP(10,19) = P(10,19);
		nextP(11,19) = P(11,19);
		nextP(12,19) = P(12,19);
		nextP(13,19) = P(13,19);
		nextP(14,19) = P(14,19);
		nextP(15,19) = P(15,19);
		nextP(16,19) = P(16,19);
		nextP(17,19) = P(17,19);
		nextP(18,19) = P(18,19);
		nextP(19,19) = P(19,19);
		nextP(1,20) = PS2*P(10,20) - PS5*P(12,20) + PS8*P(11,20) + P(1,20);
		nextP(2,20) = PS14*P(11,20) + PS19*P(12,20) - PS54*P(10,20) + P(2,20);
		nextP(3,20) = PS21*P(12,20) - PS22*P(11,20) + PS59*P(10,20) + P(3,20);
		nextP(4,20) = PS2*P(13,20) - PS34*P(2,20) - PS38*P(3,20) - PS5*P(15,20) + PS8*P(14,20) + P(4,20);
		nextP(5,20) = PS14*P(14,20) + PS19*P(15,20) - PS47*P(3,20) - PS48*P(1,20) - PS54*P(13,20) + P(5,20);
		nextP(6,20) = PS21*P(15,20) - PS22*P(14,20) - PS51*P(1,20) - PS52*P(2,20) + PS59*P(13,20) + P(6,20);
		nextP(7,20) = P(4,20)*dt + P(7,20);
		nextP(8,20) = P(5,20)*dt + P(8,20);
		nextP(9,20) = P(6,20)*dt + P(9,20);
		nextP(10,20) = P(10,20);
		nextP(11,20) = P(11,20);
		nextP(12,20) = P(12,20);
		nextP(13,20) = P(13,20);
		nextP(14,20) = P(14,20);
		nextP(15,20) = P(15,20);
		nextP(16,20) = P(16,20);
		nextP(17,20) = P(17,20);
		nextP(18,20) = P(18,20);
		nextP(19,20) = P(19,20);
		nextP(20,20) = P(20,20);
		nextP(1,21) = PS2*P(10,21) - PS5*P(12,21) + PS8*P(11,21) + P(1,21);
		nextP(2,21) = PS14*P(11,21) + PS19*P(12,21) - PS54*P(10,21) + P(2,21);
		nextP(3,21) = PS21*P(12,21) - PS22*P(11,21) + PS59*P(10,21) + P(3,21);
		nextP(4,21) = PS2*P(13,21) - PS34*P(2,21) - PS38*P(3,21) - PS5*P(15,21) + PS8*P(14,21) + P(4,21);
		nextP(5,21) = PS14*P(14,21) + PS19*P(15,21) - PS47*P(3,21) - PS48*P(1,21) - PS54*P(13,21) + P(5,21);
		nextP(6,21) = PS21*P(15,21) - PS22*P(14,21) - PS51*P(1,21) - PS52*P(2,21) + PS59*P(13,21) + P(6,21);
		nextP(7,21) = P(4,21)*dt + P(7,21);
		nextP(8,21) = P(5,21)*dt + P(8,21);
		nextP(9,21) = P(6,21)*dt + P(9,21);
		nextP(10,21) = P(10,21);
		nextP(11,21) = P(11,21);
		nextP(12,21) = P(12,21);
		nextP(13,21) = P(13,21);
		nextP(14,21) = P(14,21);
		nextP(15,21) = P(15,21);
		nextP(16,21) = P(16,21);
		nextP(17,21) = P(17,21);
		nextP(18,21) = P(18,21);
		nextP(19,21) = P(19,21);
		nextP(20,21) = P(20,21);
		nextP(21,21) = P(21,21);

		mag_noise = (mag_I_sig)^2;
		mag_bias_noise = (mag_B_sig)^2;
		% add process noise that is not from the IMU
		for i = 16: 18 
			nextP(i, i) = nextP(i, i) + mag_noise;
		end
		for i = 19: 21 
			nextP(i, i) = nextP(i, i) + mag_bias_noise;
		end

	end

	% Don't do covariance prediction on wind states unless we are using them
	if (control_status.flags.wind) 

		% calculate variances and upper diagonal covariances for wind states
		nextP(1,22) = PS2*P(10,22) - PS5*P(12,22) + PS8*P(11,22) + P(1,22);
		nextP(2,22) = PS14*P(11,22) + PS19*P(12,22) - PS54*P(10,22) + P(2,22);
		nextP(3,22) = PS21*P(12,22) - PS22*P(11,22) + PS59*P(10,22) + P(3,22);
		nextP(4,22) = PS2*P(13,22) - PS34*P(2,22) - PS38*P(3,22) - PS5*P(15,22) + PS8*P(14,22) + P(4,22);
		nextP(5,22) = PS14*P(14,22) + PS19*P(15,22) - PS47*P(3,22) - PS48*P(1,22) - PS54*P(13,22) + P(5,22);
		nextP(6,22) = PS21*P(15,22) - PS22*P(14,22) - PS51*P(1,22) - PS52*P(2,22) + PS59*P(13,22) + P(6,22);
		nextP(7,22) = P(4,22)*dt + P(7,22);
		nextP(8,22) = P(5,22)*dt + P(8,22);
		nextP(9,22) = P(6,22)*dt + P(9,22);
		nextP(10,22) = P(10,22);
		nextP(11,22) = P(11,22);
		nextP(12,22) = P(12,22);
		nextP(13,22) = P(13,22);
		nextP(14,22) = P(14,22);
		nextP(15,22) = P(15,22);
		nextP(16,22) = P(16,22);
		nextP(17,22) = P(17,22);
		nextP(18,22) = P(18,22);
		nextP(19,22) = P(19,22);
		nextP(20,22) = P(20,22);
		nextP(21,22) = P(21,22);
		nextP(22,22) = P(22,22);
		nextP(1,23) = PS2*P(10,23) - PS5*P(12,23) + PS8*P(11,23) + P(1,23);
		nextP(2,23) = PS14*P(11,23) + PS19*P(12,23) - PS54*P(10,23) + P(2,23);
		nextP(3,23) = PS21*P(12,23) - PS22*P(11,23) + PS59*P(10,23) + P(3,23);
		nextP(4,23) = PS2*P(13,23) - PS34*P(2,23) - PS38*P(3,23) - PS5*P(15,23) + PS8*P(14,23) + P(4,23);
		nextP(5,23) = PS14*P(14,23) + PS19*P(15,23) - PS47*P(3,23) - PS48*P(1,23) - PS54*P(13,23) + P(5,23);
		nextP(6,23) = PS21*P(15,23) - PS22*P(14,23) - PS51*P(1,23) - PS52*P(2,23) + PS59*P(13,23) + P(6,23);
		nextP(7,23) = P(4,23)*dt + P(7,23);
		nextP(8,23) = P(5,23)*dt + P(8,23);
		nextP(9,23) = P(6,23)*dt + P(9,23);
		nextP(10,23) = P(10,23);
		nextP(11,23) = P(11,23);
		nextP(12,23) = P(12,23);
		nextP(13,23) = P(13,23);
		nextP(14,23) = P(14,23);
		nextP(15,23) = P(15,23);
		nextP(16,23) = P(16,23);
		nextP(17,23) = P(17,23);
		nextP(18,23) = P(18,23);
		nextP(19,23) = P(19,23);
		nextP(20,23) = P(20,23);
		nextP(21,23) = P(21,23);
		nextP(22,23) = P(22,23);
		nextP(23,23) = P(23,23);

		noise_wind = (wind_vel_sig)^2;
		% add process noise that is not from the IMU
		for i = 22:23 
			nextP(i, i) = nextP(i, i) + noise_wind;
		end

    end
    k_num_states = uint8(23);
    % stop position covariance growth if our total position variance reaches 100m
	% this can happen if we lose gps for some time
    % 这里的单位是厘米
	if ((P(7, 7) + P(8, 8)) > 1e4) 
		for  i = 7: 8 
			for j = 1: k_num_states   
				nextP(i, j) = P(i, j);
				nextP(j, i) = P(j, i);
			end
		end
	end
    
    
	% covariance matrix is symmetrical, so copy upper half to lower half
	for row = 2: k_num_states
		for  column = 1 :row
			P(row, column) = nextP(column, row);
            P(column, row) = nextP(column, row);
		end
	end

	% copy variances (diagonals)
	for i = 1 : k_num_states
		P(i, i) = nextP(i, i);
	end

	% fix gross errors in the covariance matrix and ensure rows and
	% columns for un-used states are zero
	fixCovarianceErrors(false);

end