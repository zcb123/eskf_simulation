function fuseGpsHgt()
    
    global states params gps_sample_delayed 
    global gps_alt_ref hgt_sensor_offset gps_pos_innov_var gps_pos_test_ratio; 
    
    % vertical position innovation - gps measurement has opposite sign to earth z axis
	gps_pos_innov(2) = states.pos(2) + gps_sample_delayed.hgt - gps_alt_ref - hgt_sensor_offset;

	% innovation gate size
	innov_gate = fmaxf(params.baro_innov_gate, 1);

	obs_var = getGpsHeightVariance();

	% _gps_pos_test_ratio(1) is the vertical test ratio
	
	pos_offset_body = params.gps_pos_body - params.imu_pos_body;

	[gps_pos_innov_var(2),gps_pos_test_ratio(1),flag] = fuseVerticalPositionWithLevelArm(pos_offset_body, gps_pos_innov(2), innov_gate, obs_var);
    

end
