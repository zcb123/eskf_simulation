function resetGpsDriftCheckFilters()
    
    global gps_horizontal_position_drift_rate_m_s gps_vertical_position_drift_rate_m_s gps_filtered_horizontal_velocity_m_s
%     gps_velNE_filt.setZero();
% 	gps_pos_deriv_filt.setZero();

	gps_horizontal_position_drift_rate_m_s = nan;
	gps_vertical_position_drift_rate_m_s = nan;
	gps_filtered_horizontal_velocity_m_s = nan;


end