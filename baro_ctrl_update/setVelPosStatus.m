function setVelPosStatus(index,healthy)

    global fault_status time_last_imu;
    global time_last_hor_vel_fuse time_last_ver_vel_fuse time_last_hor_pos_fuse time_last_hgt_fuse
    
    switch (index) 
	case 1
		if (healthy) 
			fault_status.flags.bad_vel_N = false;
			time_last_hor_vel_fuse = time_last_imu;

		else 
			fault_status.flags.bad_vel_N = true;
		end

		

	case 2
		if (healthy) 
			fault_status.flags.bad_vel_E = false;
			time_last_hor_vel_fuse = time_last_imu;

		else 
			fault_status.flags.bad_vel_E = true;
		end

		

	case 3
		if (healthy) 
			fault_status.flags.bad_vel_D = false;
			time_last_ver_vel_fuse = time_last_imu;

		else 
			fault_status.flags.bad_vel_D = true;
		end

		

	case 4
		if (healthy) 
			fault_status.flags.bad_pos_N = false;
			time_last_hor_pos_fuse = time_last_imu;

		else 
			fault_status.flags.bad_pos_N = true;
		end

		

	case 5
		if (healthy) 
			fault_status.flags.bad_pos_E = false;
			time_last_hor_pos_fuse = time_last_imu;

		else 
			fault_status.flags.bad_pos_E = true;
		end

		

	case 6
		if (healthy) 
			fault_status.flags.bad_pos_D = false;
			time_last_hgt_fuse = time_last_imu;

		else 
			fault_status.flags.bad_pos_D = true;
		end

		
	end


end

