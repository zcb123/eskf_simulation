function setControlGPSHeight()

    global control_status;
	control_status.flags.gps_hgt = true;

	control_status.flags.baro_hgt = false;
	control_status.flags.rng_hgt = false;
	control_status.flags.ev_hgt = false;

end