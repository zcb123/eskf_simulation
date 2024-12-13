function [innov_var,test_ratio,ret]=fuseVerticalPosition(innov, innov_gate, obs_var)
    global P fault_status
    global time_last_imu;
    global vert_pos_innov_ratio vert_pos_fuse_attempt_time_us;
    k_pos_id = 7;
    innov_var = P(k_pos_id+2, k_pos_id+2) + obs_var;
	test_ratio = sq(innov) / (sq(innov_gate) * innov_var);

	vert_pos_innov_ratio = innov / sqrt(innov_var);
	vert_pos_fuse_attempt_time_us = time_last_imu;

	innov_check_pass = test_ratio <= 1;

	% if there is bad vertical acceleration data, then don't reject measurement,
	% but limit innovation to prevent spikes that could destabilise the filter
	

	if (fault_status.flags.bad_acc_vertical && ~innov_check_pass) 
		innov_limit = innov_gate * sqrtf(innov_var);
		innovation = saturation(innov, -innov_limit, innov_limit);
		innov_check_pass = true;

	else 
		innovation = innov;
	end

	if (innov_check_pass) 
		%innov_check_fail_status.flags.reject_ver_pos = false;
        
		ret = fuseVelPosHeight(innovation, innov_var, 6);
        
        return 
	else 
		%innov_check_fail_status.flags.reject_ver_pos = true;
		ret= false;
        return;
	end


end

