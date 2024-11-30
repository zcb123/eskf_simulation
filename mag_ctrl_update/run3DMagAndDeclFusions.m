function run3DMagAndDeclFusions()
    global mag_decl_cov_reset control_status;
    
    if (~mag_decl_cov_reset) 
		% After any magnetic field covariance reset event the earth field state
		% covariances need to be corrected to incorporate knowledge of the declination
		% before fusing magnetomer data to prevent rapid rotation of the earth field
		% states for the first few observations.
		fuseDeclination(0.02);
		mag_decl_cov_reset = true;
		fuseMag(mag);

	else 
		% The normal sequence is to fuse the magnetometer data first before fusing
		% declination angle at a higher uncertainty to allow some learning of
		% declination angle over time.
		fuseMag(mag);

		if (control_status.flags.mag_dec) 
			fuseDeclination(0.5);
		end
	end



end

