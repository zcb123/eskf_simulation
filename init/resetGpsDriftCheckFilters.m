function resetGpsDriftCheckFilters()
    
    global gps_horizontal_position_drift_rate_m_s gps_vertical_position_drift_rate_m_s gps_filtered_horizontal_velocity_m_s
    global gps_velNE_filt gps_pos_deriv_filt;
    
    gps_velNE_filt = zeros(2,1);
	gps_pos_deriv_filt = zeros(3,1);

	gps_horizontal_position_drift_rate_m_s = nan;
	gps_vertical_position_drift_rate_m_s = nan;
	gps_filtered_horizontal_velocity_m_s = nan;


end