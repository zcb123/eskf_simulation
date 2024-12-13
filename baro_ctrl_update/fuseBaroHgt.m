function fuseBaroHgt()
    global states control_status params;
    global baro_sample_delayed baro_hgt_offset;
    global baro_b_est;
	% vertical position innovation - baro measurement has opposite sign to earth z axis
	
    unbiased_baro = baro_sample_delayed.hgt - baro_b_est.getBias();

	baro_hgt_innov = states.pos(3) + unbiased_baro - baro_hgt_offset;

	% Compensate for positive static pressure transients (negative vertical position innovations)
	% caused by rotor wash ground interaction by applying a temporary deadzone to baro innovations.
	if (control_status.flags.gnd_effect && (params.gnd_effect_deadzone > 0)) 

		deadzone_start = 0.0;
		deadzone_end = deadzone_start + params.gnd_effect_deadzone;

		if (baro_hgt_innov < -deadzone_start) 
			if (baro_hgt_innov <= -deadzone_end) 

				baro_hgt_innov = baro_hgt_innov + deadzone_end;

            else 

				baro_hgt_innov = -deadzone_start;
                
			end
		end
	end

	% innovation gate size
	innov_gate = fmaxf(params.baro_innov_gate, 1);

	% observation variance - user parameter defined
	obs_var = sq(fmaxf(params.baro_noise, 0.01));

   
    disp('fuse baro height');
	[baro_hgt_innov_var,baro_hgt_test_ratio,flag] = fuseVerticalPosition(baro_hgt_innov, innov_gate, obs_var);
			     
    
    assignin("base",'baro_hgt_test_ratio',baro_hgt_test_ratio);


end