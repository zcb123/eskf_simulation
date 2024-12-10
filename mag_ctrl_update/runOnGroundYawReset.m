function runOnGroundYawReset()
    global control_status params dt_ekf_avg;
    global mag_yaw_reset_req mag_inhibit_yaw_reset_req is_yaw_fusion_inhibited;
    
    if (mag_yaw_reset_req && ~is_yaw_fusion_inhibited) 

        if canResetMagHeading()
            has_realigned_yaw = resetMagHeading(false,false);
        else
            has_realigned_yaw = false;
        end
	
		if (has_realigned_yaw) 
			mag_yaw_reset_req = false;
			control_status.flags.yaw_align = true;

			% Handle the special case where we have not been constraining yaw drift or learning yaw bias due
			% to assumed invalid mag field associated with indoor operation with a downwards looking flow sensor.
			if (mag_inhibit_yaw_reset_req) 

				mag_inhibit_yaw_reset_req = false;
				% Zero the yaw bias covariance and set the variance to the initial alignment uncertainty
                %P(12,12) = sq(params.switch_on_gyro_bias * dt_ekf_avg);
				uncorrelateCovarianceSetVariance(1,12, sq(params.switch_on_gyro_bias * dt_ekf_avg));
			end
		end
	end
end

