function fuseGpsVelPos()
    global states control_status params gps_sample_delayed;
    global gps_vel_test_ratio gps_pos_test_ratio;
    gps_pos_obs_var =single([ 0 0 0]);

	lower_limit = max(params.gps_pos_noise, 0.01);

	if (isOtherSourceOfHorizontalAidingThan(control_status.flags.gps)) 
% 		% if we are using other sources of aiding, then relax the upper observation
% 		% noise limit which prevents bad GPS perturbing the position estimate
% 		gps_pos_obs_var(1) = gps_pos_obs_var(2) = sq(max(gps_sample_delayed.hacc, lower_limit));
% 
    else 
		% if we are not using another source of aiding, then we are reliant on the GPS
		% observations to constrain attitude errors and must limit the observation noise value.
	    upper_limit = max(params.pos_noaid_noise, lower_limit);   
        hacc = saturation(gps_sample_delayed.hacc, lower_limit, upper_limit);
	    gps_pos_obs_var(1) = hacc^2;
        gps_pos_obs_var(2) = gps_pos_obs_var(1);
	end



	vel_var = sq(gps_sample_delayed.sacc);
	gps_vel_obs_var = single([vel_var  vel_var  vel_var*sq(2.5)]);

	% calculate innovations
	gps_vel_innov = states.vel - gps_sample_delayed.vel;
	gps_pos_innov_xy = states.pos(1:2,1) - gps_sample_delayed.pos(1:2,1);

	% set innovation gate size
	pos_innov_gate = max(params.gps_pos_innov_gate, 2);
	vel_innov_gate = max(params.gps_vel_innov_gate, 2);

	% fuse GPS measurement
	%fuseHorizontalVelocity(gps_vel_innov, vel_innov_gate, gps_vel_obs_var, _gps_vel_innov_var, gps_vel_test_ratio);
	%fuseVerticalVelocity(gps_vel_innov, vel_innov_gate, gps_vel_obs_var, _gps_vel_innov_var, gps_vel_test_ratio);
	%fuseHorizontalPosition(gps_pos_innov, pos_innov_gate, gps_pos_obs_var, gps_pos_innov_var, gps_pos_test_ratio);*/
	pos_offset_body = params.gps_pos_body - params.imu_pos_body;
	
	[gps_vel_innov_var,vel_test_ratio,vel_xyz_update] = fuseVelocityWithLevelArm(params,pos_offset_body, gps_vel_innov, vel_innov_gate, gps_vel_obs_var);
%   fuseVelocityWithLevelArm_Matrix(pos_offset_body, gps_vel_innov, vel_innov_gate, gps_vel_obs_var);
	gps_vel_test_ratio(1) = max(vel_test_ratio(1), vel_test_ratio(2));
	gps_vel_test_ratio(2) = vel_test_ratio(3);

    [gps_pos_innov_var,pos_test_ratio,pos_xy_update] = fuseHorizontalPositionWithLevelArm(params,pos_offset_body, gps_pos_innov_xy, pos_innov_gate, gps_pos_obs_var,false);
	gps_pos_test_ratio(1) = max(pos_test_ratio(1), pos_test_ratio(2));
end