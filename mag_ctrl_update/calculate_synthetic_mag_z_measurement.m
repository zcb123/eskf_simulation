function res = calculate_synthetic_mag_z_measurement(mag_meas,mag_earth_predicted)


    global R_to_earth;
    
    mag_z_abs = sqrtf(max(sq(norm(mag_earth_predicted))-sq(mag_meas(1)) - sq(mag_meas(2)), 0));

	%calculate sign of synthetic magnetomter Z component based on the sign of the predicted magnetomer Z component
	mag_z_body_pred = sum(mag_earth_predicted.*R_to_earth(:,3));

    if mag_z_body_pred<0
        res = -mag_z_abs;
    else
        res = mag_z_abs;
    end
	

end


