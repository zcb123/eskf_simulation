function runInAirYawReset()
    
    global control_status params dt_ekf_avg P;
    global mag_yaw_reset_req is_yaw_fusion_inhibited mag_inhibit_yaw_reset_req;

    if (mag_yaw_reset_req && ~is_yaw_fusion_inhibited) 
	    has_realigned_yaw = false;

		if (control_status.flags.gps && control_status.flags.fixed_wing) 
			
            disp("fixed wing");

        elseif(canResetMagHeading()) 

			has_realigned_yaw = resetMagHeading(true,true);
            
        end

		if (has_realigned_yaw) 
			mag_yaw_reset_req = false;
			control_status.flags.yaw_align = true;
			control_status.flags.mag_aligned_in_flight = true;

			% Handle the special case where we have not been constraining yaw drift or learning yaw bias due
			% to assumed invalid mag field associated with indoor operation with a downwards looking flow sensor.
			if (mag_inhibit_yaw_reset_req) 
				mag_inhibit_yaw_reset_req = false;
				% Zero the yaw bias covariance and set the variance to the initial alignment uncertainty
				%_P.uncorrelateCovarianceSetVariance<1>(11, sq(_params.switch_on_gyro_bias * _dt_ekf_avg));
                P(12,:) = 0;
                P(:,12) = 0;
                P(12,12) = sq(params.switch_on_gyro_bias * dt_ekf_avg);
			end
		end

	end





end

