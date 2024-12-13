function ret = fuseVelPosHeight(innov, innov_var, obs_index)
    global P;
    Kfusion = zeros(23,1);  % Kalman gain vector for any single observation - sequential fusion is used.
	state_index = obs_index + 3;  % we start with vx and this is the 4. state

	% calculate kalman gain K = PHS, where S = 1/innovation variance
	for row = 1:23 
		Kfusion(row) = P(row, state_index) / innov_var;%在前面加了P(state_index,state_index)
	end

	KHP = zeros(23,23);

	for row = 1:23
		for column = 1:23 
			KHP(row, column) = Kfusion(row) * P(state_index, column);
		end
	end

	% if the covariance correction will result in a negative variance, then
	% the covariance matrix is unhealthy and must be corrected
	healthy = true;

	for i = 1:23
		if (P(i, i) < KHP(i, i)) 
			% zero rows and columns
			uncorrelateCovarianceSetVariance(1,i,0);
			healthy = false;
		end
	end

	setVelPosStatus(obs_index, healthy);

	if (healthy) 
		% apply the covariance corrections
		P = P - KHP;

		fixCovarianceErrors(true);

		switch (obs_index)
			
		
		case 1
			fuse(Kfusion, innov);
			

		case 2
			fuse(Kfusion, innov);
			
		
		case 3
			fuse(Kfusion, innov);
			

		case 4
			fuse(Kfusion, innov);
			

		case 5
			fuse(Kfusion, innov);
			

		case 6
			fuse(Kfusion, innov);
			    
		end
		% apply the state corrections
		%fuse(Kfusion, innov);
        ret = true;
		return;
	end

	ret = false;

end


