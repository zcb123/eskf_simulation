function gps_alt_var = getGpsHeightVariance(gps_sample_delayed)
    
    global params;

    lower_limit = fmaxf(1.5 * params.gps_pos_noise, 0.01);
	upper_limit = fmaxf(1.5 * params.pos_noaid_noise, lower_limit);
	gps_alt_var = sq(saturation(gps_sample_delayed.vacc, lower_limit, upper_limit));


end


