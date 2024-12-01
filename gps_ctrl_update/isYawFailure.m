function ret = isYawFailure()
    global yawEstimator;
    global R_to_earth;
    if ~yawEstimator.ekf_gsf_vel_fuse_started
        ret = false;
        return
    end
    
    if ~yawEstimator.gsf_yaw_variance < (params.EKFGSF_yaw_err_max^2)
        ret = false;
        return
    end

    euler_yaw = getEulerYaw(R_to_earth);
	yaw_error = wrap_pn_pi(euler_yaw - yawEstimator.getYaw());

	ret = abs(yaw_error) >(25/57.3);
end

