function stopGpsFusion()

    global control_status;
    global gps_pos_innov gps_pos_innov_var gps_pos_test_ratio ;
    global gps_vel_innov gps_vel_innov_var gps_vel_test_ratio ;
    if control_status.flags.gps
		%stopGpsPosFusion();
		%stopGpsVelFusion();

        control_status.flags.gps = false;
        
        if control_status.flags.gps_hgt                 %GPS数据不更新了，自动切换成气压计
             

            startBaroHgtFusion();

            control_status.flags.baro_hgt = true;
	        control_status.flags.gps_hgt = false;
	        control_status.flags.rng_hgt = false;
	        control_status.flags.ev_hgt = false;

        end

        gps_pos_innov = zeros(3,1);
        gps_pos_innov_var = zeros(3,1);
        gps_pos_test_ratio = zeros(3,1);
        
        gps_vel_innov = zeros(3,1);
	    gps_vel_innov_var = zeros(3,1);
	    gps_vel_test_ratio= zeros(3,1);

	end

	if control_status.flags.gps_yaw 
		control_status.flags.gps_yaw = false;
	end



end


