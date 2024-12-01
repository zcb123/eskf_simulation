function runMagAndMagDeclFusions(mag)
    
    global states params control_status R_to_earth;

    if (control_status.flags.mag_3D)

		run3DMagAndDeclFusions(mag);

    elseif (control_status.flags.mag_hdg) 
		% Rotate the measurements into earth frame using the zero yaw angle
		R_to_earth = updateEuler321YawInRotMat(0, R_to_earth); 

		mag_earth_pred = R_to_earth*(mag - states.mag_B);

		% the angle of the projection onto the horizontal gives the yaw angle
		measured_hdg = -atan2(mag_earth_pred(2), mag_earth_pred(1)) + getMagDeclination();

		fuseHeading(measured_hdg, sq(params.mag_heading_noise));

    end



end


