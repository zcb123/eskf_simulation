function [obj,res] = updateEKF(obj,model_index)

    % set observation variance from accuracy estimate supplied by GPS and apply a sanity check minimum
	velObsVar = sq(max(obj.vel_accuracy, 0.01));        %限制了最小值 0.01

	% calculate velocity observation innovations
	obj.ekf_gsf(model_index,1).innov(1) = obj.ekf_gsf(model_index,1).X(1) - obj.vel_NE(1);
	obj.ekf_gsf(model_index,1).innov(2) = obj.ekf_gsf(model_index,1).X(2) - obj.vel_NE(2);

	% Use temporary variables for covariance elements to reduce verbosity of auto-code expressions
	P00 = obj.ekf_gsf(model_index,1).P(1,1);
	P01 = obj.ekf_gsf(model_index,1).P(1,2);
	P02 = obj.ekf_gsf(model_index,1).P(1,3);
	P11 = obj.ekf_gsf(model_index,1).P(2,2);
	P12 = obj.ekf_gsf(model_index,1).P(2,3);
	P22 = obj.ekf_gsf(model_index,1).P(3,3);

	% optimized auto generated code from SymPy script src/lib/ecl/EKF/python/ekf_derivation/main.py
	t0 = P01*P01;
	t1 = -t0;
	t2 = P00*P11 + P00*velObsVar + P11*velObsVar + t1 + velObsVar*velObsVar;
	if (abs(t2) < 1e-6) 
        res = false;
		return    
	end
	t3 = 1.0/t2;
	t4 = P11 + velObsVar;
	t5 = P01*t3;
	t6 = -t5;
	t7 = P00 + velObsVar;
	t8 = P00*t4 + t1;
	t9 = t5*velObsVar;
	t10 = P11*t7;
	t11 = t1 + t10;
	t12 = P01*P12;
	t13 = P02*t4;
	t14 = P01*P02;
	t15 = P12*t7;
	t16 = t0*velObsVar;
	t17 = powf(t2,-2);
	t18 = t4*velObsVar + t8;
	t19 = t17*t18;
	t20 = t17*(t16 + t7*t8);
	t21 = t0 - t10;
	t22 = t17*t21;
	t23 = t14 - t15;
	t24 = P01*t23;
	t25 = t12 - t13;
	t26 = t16 - t21*t4;
	t27 = t17*t26;
	t28 = t11 + t7*velObsVar;
	t30 = t17*t28;
	t31 = P01*t25;
	t32 = t23*t4 + t31;
	t33 = t17*t32;
	t35 = t24 + t25*t7;
	t36 = t17*t35;

	obj.ekf_gsf(model_index,1).S_det_inverse = t3;

	obj.ekf_gsf(model_index,1).S_inverse(1,1) = t3*t4;
	obj.ekf_gsf(model_index,1).S_inverse(1,2) = t6;
	obj.ekf_gsf(model_index,1).S_inverse(2,2) = t3*t7;
	obj.ekf_gsf(model_index,1).S_inverse(2,1) = obj.ekf_gsf(model_index,1).S_inverse(1,2);
%     if obj.ekf_gsf(model_index,1).S_inverse(1,2) > 1e3
%         debug = 1;
%     end
	K = zeros(3,2);
	K(1,1) = t3*t8;
	K(2,1) = t9;
	K(3,1) = t3*(-t12 + t13);
	K(1,2) = t9;
	K(2,2) = t11*t3;
	K(3,2) = t3*(-t14 + t15);

	obj.ekf_gsf(model_index,1).P(1,1) = P00 - t16*t19 - t20*t8;
	obj.ekf_gsf(model_index,1).P(1,2) = P01*(t18*t22 - t20*velObsVar + 2);
	obj.ekf_gsf(model_index,1).P(2,2) = P11 - t16*t30 + t22*t26;
	obj.ekf_gsf(model_index,1).P(1,3) = P02 + t19*t24 + t20*t25;
	obj.ekf_gsf(model_index,1).P(2,3) = P12 + t23*t27 + t30*t31;
	obj.ekf_gsf(model_index,1).P(3,3) = P22 - t23*t33 - t25*t36;
	obj.ekf_gsf(model_index,1).P(2,1) = obj.ekf_gsf(model_index,1).P(1,2);
	obj.ekf_gsf(model_index,1).P(3,1) = obj.ekf_gsf(model_index,1).P(1,3);
	obj.ekf_gsf(model_index,1).P(3,2) = obj.ekf_gsf(model_index,1).P(2,3);

	% constrain variances
	min_var = 1e-6;

	for index = 1:3 
		obj.ekf_gsf(model_index,1).P(index, index) = max(obj.ekf_gsf(model_index,1).P(index, index), min_var);
	end

	% test ratio = transpose(innovation) * inverse(innovation variance) * innovation =[1x2] * [2,2] * [2,1] = [1,1]
	test_ratio = obj.ekf_gsf(model_index,1).innov' * (obj.ekf_gsf(model_index,1).S_inverse * obj.ekf_gsf(model_index,1).innov);
    assignin("base","test_ratio",test_ratio);
	% Perform a chi-square innovation consistency test and calculate a compression scale factor
	% that limits the magnitude of innovations to 5-sigma
	% If the test ratio is greater than 25 (5 Sigma) then reduce the length of the innovation vector to clip it at 5-Sigma
	% This protects from large measurement spikes
    if(test_ratio > 25)
	    innov_comp_scale_factor = sqrt(25.0 / test_ratio);
    else
        innov_comp_scale_factor = 1;
    end
        
	% Correct the state vector and capture the change in yaw angle
	oldYaw = obj.ekf_gsf(model_index,1).X(3);

	obj.ekf_gsf(model_index,1).X = obj.ekf_gsf(model_index,1).X - (K * obj.ekf_gsf(model_index,1).innov) * innov_comp_scale_factor;

	yawDelta = obj.ekf_gsf(model_index,1).X(3) - oldYaw;
    
	% apply the change in yaw angle to the AHRS
	% take advantage of sparseness(稀疏) in the yaw rotation matrix
	cosYaw = cos(yawDelta);
	sinYaw = sin(yawDelta);
	R_prev00 = obj.ahrs_ekf_gsf(model_index,1).R(1, 1);
	R_prev01 = obj.ahrs_ekf_gsf(model_index,1).R(1, 2);
	R_prev02 = obj.ahrs_ekf_gsf(model_index,1).R(1, 3);

	obj.ahrs_ekf_gsf(model_index,1).R(1, 1) = R_prev00 * cosYaw - obj.ahrs_ekf_gsf(model_index,1).R(2, 1) * sinYaw;
	obj.ahrs_ekf_gsf(model_index,1).R(1, 2) = R_prev01 * cosYaw - obj.ahrs_ekf_gsf(model_index,1).R(2, 2) * sinYaw;
	obj.ahrs_ekf_gsf(model_index,1).R(1, 3) = R_prev02 * cosYaw - obj.ahrs_ekf_gsf(model_index,1).R(2, 3) * sinYaw;
	obj.ahrs_ekf_gsf(model_index,1).R(2, 1) = R_prev00 * sinYaw + obj.ahrs_ekf_gsf(model_index,1).R(2, 1) * cosYaw;
	obj.ahrs_ekf_gsf(model_index,1).R(2, 2) = R_prev01 * sinYaw + obj.ahrs_ekf_gsf(model_index,1).R(2, 2) * cosYaw;
	obj.ahrs_ekf_gsf(model_index,1).R(2, 3) = R_prev02 * sinYaw + obj.ahrs_ekf_gsf(model_index,1).R(2, 3) * cosYaw;

	res = true;

end

