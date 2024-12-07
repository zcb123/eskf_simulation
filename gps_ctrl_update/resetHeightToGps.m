function resetHeightToGps()

    global states gps_sample_delayed gps_alt_ref baro_hgt_offset;
    z_pos_before_reset = states.pos(3);
	resetVerticalPositionTo(-gps_sample_delayed.hgt + gps_alt_ref);

	% the state variance is the same as the observation
	uncorrelateCovarianceSetVariance(1,9, getGpsHeightVariance());

	% adjust the baro offset
	baro_hgt_offset = baro_hgt_offset + states.pos(3) - z_pos_before_reset;

end

