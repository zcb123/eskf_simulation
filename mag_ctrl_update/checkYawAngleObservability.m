function checkYawAngleObservability()

    global yaw_angle_observable accel_lpf_NE control_status params;


    % Check if there has been enough change in horizontal velocity to make yaw observable
	% Apply hysteresis to check to avoid rapid toggling

    %     _yaw_angle_observable = _yaw_angle_observable
    % 				? _accel_lpf_NE.norm() > _params.mag_acc_gate
    % 				: _accel_lpf_NE.norm() > 2.0f * _params.mag_acc_gate;

    if yaw_angle_observable
        if norm(accel_lpf_NE) > params.mag_acc_gate
            yaw_angle_observable = true;
        else
            yaw_angle_observable = false;
        end
    else
        if norm(accel_lpf_NE) > 2*params.mag_acc_gate
            yaw_angle_observable = true;
        else
            yaw_angle_observable = false;
        end
    end



    yaw_angle_observable = yaw_angle_observable && control_status.flags.gps;
    
end

