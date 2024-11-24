function obj = ahrsAlignYaw(obj)

       	% Align yaw angle for each model
	for model_index = 1:5
		R = obj.ahrs_ekf_gsf(model_index,1).R;
		yaw = wrap_pi(obj.ekf_gsf(model_index,1).X(3));
		R = updateYawInRotMat(yaw, R);

		obj.ahrs_ekf_gsf(model_index,1).aligned = true;
        
    end

end

