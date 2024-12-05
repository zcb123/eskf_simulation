function res = getGpsHeightVariance()

    global params gps_sample_delayed;
    lower_limit = fmaxf(1.5* params.gps_pos_noise, 0.01);
	upper_limit = fmaxf(1.5* params.pos_noaid_noise, lower_limit);
    if isempty(gps_sample_delayed)
        gps_sample_delayed.time_us = 0;
        gps_sample_delayed.vacc = 0.5;
    end

	gps_alt_var = sq(saturation(gps_sample_delayed.vacc, lower_limit, upper_limit));
	res = gps_alt_var;


end