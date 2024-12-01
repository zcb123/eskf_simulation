function checkHaglYawResetReq()
    global control_status states;
    global mag_yaw_reset_req
    if (~control_status.flags.mag_aligned_in_flight) 
		% Check if height has increased sufficiently to be away from ground magnetic anomalies
		% and request a yaw reset if not already requested.
		mag_anomalies_max_hagl = 1.5;
		above_mag_anomalies = (getTerrainVPos() - states.pos(3)) > mag_anomalies_max_hagl;
		mag_yaw_reset_req = mag_yaw_reset_req || above_mag_anomalies;
	end

end