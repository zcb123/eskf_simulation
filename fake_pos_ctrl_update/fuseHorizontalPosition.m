function [innov_var,test_ratio,ret] = fuseHorizontalPosition(innov,innov_gate,obs_var,inhibit_gate)
    
    global P;
    global innov_check_fail_status;
    k_pos_id = 7;
    innov_var(1) = P(k_pos_id, k_pos_id) + obs_var(1);
	innov_var(2) = P(k_pos_id+1, k_pos_id+1) + obs_var(2);
	test_ratio(1) = fmaxf(sq(innov(1)) / (sq(innov_gate) * innov_var(1)),...
			      sq(innov(2)) / (sq(innov_gate) * innov_var(2)));

	innov_check_pass = test_ratio(1) <= 1;

	if (innov_check_pass || inhibit_gate) 
		if (inhibit_gate && test_ratio(1) > sq(100 / innov_gate)) 
			% always protect against extreme values that could result in a NaN
			ret = false;
            return ;
		end

		innov_check_fail_status.flags.reject_hor_pos = false;

		fuse_x = fuseVelPosHeight(innov(1), innov_var(1), 4);
		fuse_y = fuseVelPosHeight(innov(2), innov_var(2), 5);

		ret = fuse_x && fuse_y;

	else 
		innov_check_fail_status.flags.reject_hor_pos = true;
		ret = false;
    end

end