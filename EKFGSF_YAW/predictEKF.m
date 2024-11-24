function predictEKF(const uint8_t model_index)
            ahrsPredict(model_index);

	// we don't start running the EKF part of the algorithm until there are regular velocity observations
	if (!_ekf_gsf_vel_fuse_started) {
		return;
	}

	// Calculate the yaw state using a projection onto the horizontal that avoids gimbal lock
	_ekf_gsf[model_index].X(2) = getEulerYaw(_ahrs_ekf_gsf[model_index].R);

	// calculate delta velocity in a horizontal front-right frame
	const Vector3f del_vel_NED = _ahrs_ekf_gsf[model_index].R * _delta_vel;
	const float cos_yaw = cosf(_ekf_gsf[model_index].X(2));
	const float sin_yaw = sinf(_ekf_gsf[model_index].X(2));
	const float dvx =   del_vel_NED(0) * cos_yaw + del_vel_NED(1) * sin_yaw;
	const float dvy = - del_vel_NED(0) * sin_yaw + del_vel_NED(1) * cos_yaw;

	// sum delta velocities in earth frame:
	_ekf_gsf[model_index].X(0) += del_vel_NED(0);
	_ekf_gsf[model_index].X(1) += del_vel_NED(1);

	// predict covariance - equations generated using EKF/python/gsf_ekf_yaw_estimator/main.py

	// Local short variable name copies required for readability
	const float &P00 = _ekf_gsf[model_index].P(0,0);
	const float &P01 = _ekf_gsf[model_index].P(0,1);
	const float &P02 = _ekf_gsf[model_index].P(0,2);
	const float &P11 = _ekf_gsf[model_index].P(1,1);
	const float &P12 = _ekf_gsf[model_index].P(1,2);
	const float &P22 = _ekf_gsf[model_index].P(2,2);
	const float &psi = _ekf_gsf[model_index].X(2);

	// Use fixed values for delta velocity and delta angle process noise variances
	const float dvxVar = sq(_accel_noise * _delta_vel_dt); // variance of forward delta velocity - (m/s)^2
	const float dvyVar = dvxVar; // variance of right delta velocity - (m/s)^2
	const float dazVar = sq(_gyro_noise * _delta_ang_dt); // variance of yaw delta angle - rad^2

	// optimized auto generated code from SymPy script src/lib/ecl/EKF/python/ekf_derivation/main.py
	const float S0 = cosf(psi);
	const float S1 = ecl::powf(S0, 2);
	const float S2 = sinf(psi);
	const float S3 = ecl::powf(S2, 2);
	const float S4 = S0*dvy + S2*dvx;
	const float S5 = P02 - P22*S4;
	const float S6 = S0*dvx - S2*dvy;
	const float S7 = S0*S2;
	const float S8 = P01 + S7*dvxVar - S7*dvyVar;
	const float S9 = P12 + P22*S6;

	_ekf_gsf[model_index].P(0,0) = P00 - P02*S4 + S1*dvxVar + S3*dvyVar - S4*S5;
	_ekf_gsf[model_index].P(0,1) = -P12*S4 + S5*S6 + S8;
	_ekf_gsf[model_index].P(1,1) = P11 + P12*S6 + S1*dvyVar + S3*dvxVar + S6*S9;
	_ekf_gsf[model_index].P(0,2) = S5;
	_ekf_gsf[model_index].P(1,2) = S9;
	_ekf_gsf[model_index].P(2,2) = P22 + dazVar;

	// covariance matrix is symmetrical, so copy upper half to lower half
	_ekf_gsf[model_index].P(1,0) = _ekf_gsf[model_index].P(0,1);
	_ekf_gsf[model_index].P(2,0) = _ekf_gsf[model_index].P(0,2);
	_ekf_gsf[model_index].P(2,1) = _ekf_gsf[model_index].P(1,2);

	// constrain variances
	const float min_var = 1e-6f;

	for (unsigned index = 0; index < 3; index++) {
		_ekf_gsf[model_index].P(index, index) = fmaxf(_ekf_gsf[model_index].P(index, index), min_var);
	}

        end