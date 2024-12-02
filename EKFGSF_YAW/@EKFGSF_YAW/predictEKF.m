function obj = predictEKF(obj,model_index)
    % generate an attitude reference using IMU data
	obj.ahrsPredict(model_index);       %预测航向角

	% we don't start running the EKF part of the algorithm until there are regular velocity observations
	if ~obj.ekf_gsf_vel_fuse_started
		return;

	end

	% Calculate the yaw state using a projection onto the horizontal that avoids gimbal lock
	obj.ekf_gsf(model_index,1).X(3) = getEulerYaw(obj.ahrs_ekf_gsf(model_index,1).R);

	% calculate delta velocity in a horizontal front-right frame
	del_vel_NED = obj.ahrs_ekf_gsf(model_index,1).R * obj.delta_vel;
	cos_yaw = cos(obj.ekf_gsf(model_index,1).X(3));
	sin_yaw = sin(obj.ekf_gsf(model_index,1).X(3));
	dvx =   del_vel_NED(1) * cos_yaw + del_vel_NED(2) * sin_yaw;
	dvy = - del_vel_NED(1) * sin_yaw + del_vel_NED(2) * cos_yaw;

	% sum delta velocities in earth frame:  地理系速度预测
	obj.ekf_gsf(model_index,1).X(1) = obj.ekf_gsf(model_index,1).X(1) + del_vel_NED(1);
	obj.ekf_gsf(model_index,1).X(2) = obj.ekf_gsf(model_index,1).X(2) + del_vel_NED(2);

	% predict covariance - equations generated using EKF/python/gsf_ekf_yaw_estimator/main.py

	% Local short variable name copies required for readability
	P00 = obj.ekf_gsf(model_index,1).P(1,1);
	P01 = obj.ekf_gsf(model_index,1).P(1,2);
	P02 = obj.ekf_gsf(model_index,1).P(1,3);
	P11 = obj.ekf_gsf(model_index,1).P(2,2);
	P12 = obj.ekf_gsf(model_index,1).P(2,3);
	P22 = obj.ekf_gsf(model_index,1).P(3,3);

	psi = obj.ekf_gsf(model_index,1).X(3);

	% Use fixed values for delta velocity and delta angle process noise variances
	dvxVar = sq(obj.accel_noise * obj.delta_vel_dt); % variance of forward delta velocity - (m/s)^3
	dvyVar = dvxVar; % variance of right delta velocity - (m/s)^3
	dazVar = sq(obj.gyro_noise * obj.delta_ang_dt); % variance of yaw delta angle - rad^3

	% optimized auto generated code from SymPy script src/lib/ecl/EKF/python/ekf_derivation/main.py
	S0 = cos(psi);
	S1 = S0*S0;
	S2 = sin(psi);
	S3 = S2*S2;
	S4 = S0*dvy + S2*dvx;
	S5 = P02 - P22*S4;
	S6 = S0*dvx - S2*dvy;
	S7 = S0*S2;
	S8 = P01 + S7*dvxVar - S7*dvyVar;
	S9 = P12 + P22*S6;

	obj.ekf_gsf(model_index,1).P(1,1) = P00 - P02*S4 + S1*dvxVar + S3*dvyVar - S4*S5;
	obj.ekf_gsf(model_index,1).P(1,2) = -P12*S4 + S5*S6 + S8;
	obj.ekf_gsf(model_index,1).P(2,2) = P11 + P12*S6 + S1*dvyVar + S3*dvxVar + S6*S9;
	obj.ekf_gsf(model_index,1).P(1,3) = S5;
	obj.ekf_gsf(model_index,1).P(2,3) = S9;
	obj.ekf_gsf(model_index,1).P(3,3) = P22 + dazVar;

	% covariance matrix is symmetrical, so copy upper half to lower half
	obj.ekf_gsf(model_index,1).P(2,1) = obj.ekf_gsf(model_index,1).P(1,2);
	obj.ekf_gsf(model_index,1).P(3,1) = obj.ekf_gsf(model_index,1).P(1,3);
	obj.ekf_gsf(model_index,1).P(3,2) = obj.ekf_gsf(model_index,1).P(2,3);

	% constrain variances
	min_var = 1e-6;

	for index = 1: 3
        %这里限制了最小值min_var
		obj.ekf_gsf(model_index,1).P(index, index) = max(obj.ekf_gsf(model_index,1).P(index, index), min_var);
	end

end