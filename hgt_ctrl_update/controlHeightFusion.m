function controlHeightFusion()

    global control_status params;
    global gps_checks_passed gps_hgt_accurate baro_hgt_faulty;
    global baro_data_ready gps_data_ready;
    global baro_hgt_intermittent gps_intermittent;

    do_range_aid = false;
    switch (params.vdist_sensor_type) 
	
	case 0
		
	case 2

	case 1


		if (do_range_aid) 
			if (~control_status_prev.flags.rng_hgt && range_sensor.isDataHealthy()) 
				disp("startRngAidHgtFusion();");
			end

		else 
			if (~control_status.flags.gps_hgt) 
				if (~gps_intermittent && gps_checks_passed && gps_hgt_accurate) 
					%In fallback mode and GPS has recovered so start using it
					startGpsHgtFusion();

                elseif (~control_status.flags.baro_hgt && ~baro_hgt_faulty && ~baro_hgt_intermittent) 
					
					startBaroHgtFusion();
				end
			end
		end


	case 3

		
	end


    updateBaroHgtBias();
	updateBaroHgtOffset();
	updateGroundEffect();


    
    if (control_status.flags.baro_hgt) 
        if baro_data_ready && ~baro_hgt_faulty
            fuseBaroHgt();
        end
    elseif control_status.flags.gps_hgt && gps_hgt_accurate
        if gps_data_ready
            fuseGpsHgt();
        end
    end

end


