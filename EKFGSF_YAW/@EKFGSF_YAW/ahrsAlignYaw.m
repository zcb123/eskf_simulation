function obj = ahrsAlignYaw(obj)

       	% Align yaw angle for each model
	for model_index = 1:5
		R = obj.ahrs_ekf_gsf(model_index,1).R;
		yaw = wrap_pn_pi(obj.ekf_gsf(model_index,1).X(3));
		R = updateYawInRotMat(yaw, R);          %返回321旋转的旋转矩阵;

		obj.ahrs_ekf_gsf(model_index,1).aligned = true;

    end

end

