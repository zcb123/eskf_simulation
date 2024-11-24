function obj = initialiseEKFGSF(obj)
    obj.gsf_yaw = 0.0;
	obj.ekf_gsf_vel_fuse_started = logical(false);
	obj.gsf_yaw_variance = (pi/2) * pi/2;
	obj.model_weights = ones(5,1)*(1/5);  % All filter models start with the same weight

% 	memset(&obj.ekf_gsf, 0, sizeof(obj.ekf_gsf));
	yaw_increment = 2.0 * pi / 5;

	for model_index = 1 :5
		% evenly space initial yaw estimates in the region between +-Pi
		obj.ekf_gsf(model_index,1).X(3) = -pi + (0.5 * yaw_increment) + (model_index * yaw_increment);

		% take velocity states and corresponding variance from last measurement
		obj.ekf_gsf(model_index,1).X(1) = obj.vel_NE(1);
		obj.ekf_gsf(model_index,1).X(2) = obj.vel_NE(2);
		obj.ekf_gsf(model_index,1).P(1, 1) = sq(obj.vel_accuracy);
		obj.ekf_gsf(model_index,1).P(2, 2) = obj.ekf_gsf(model_index,1).P(1, 1);

		% use half yaw interval for yaw uncertainty
		obj.ekf_gsf(model_index,1).P(3, 3) = sq(0.5 * yaw_increment);
    end
    
end