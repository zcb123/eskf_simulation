function fuseGpsHgt()

    // vertical position innovation - gps measurement has opposite sign to earth z axis
	_gps_pos_innov(2) = _state.pos(2) + _gps_sample_delayed.hgt - _gps_alt_ref - _hgt_sensor_offset;

	// innovation gate size
	float innov_gate = fmaxf(_params.baro_innov_gate, 1.f);

	float obs_var = getGpsHeightVariance();

	// _gps_pos_test_ratio(1) is the vertical test ratio
	/*fuseVerticalPosition(_gps_pos_innov(2), innov_gate, obs_var,
			     _gps_pos_innov_var(2), _gps_pos_test_ratio(1));*/
	const Vector3f pos_offset_body = _params.gps_pos_body - _params.imu_pos_body;
	fuseVerticalPositionWithLevelArm(pos_offset_body, _gps_pos_innov(2), innov_gate, obs_var,
			     _gps_pos_innov_var(2), _gps_pos_test_ratio(1));
    

end

