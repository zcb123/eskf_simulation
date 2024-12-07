function is_healthy = measurementUpdate(K,variance,innovation)

        global P accel_bias_inhibit;
        k_vel_bias_id = 12;
        for i = 1: 3 
			if (accel_bias_inhibit(i)) 
				K(k_vel_bias_id + i) = 0;
			end
		end

		% apply covariance correction via P_new = (I -K*H)*P
		% first calculate expression for KHP
		% then calculate P - KHP
		%const SquareMatrix23f KHP = computeKHP(K, H);
		KHP = zeros(23,23);
		for row = 1 :23
			% 一行一行计算的,高斯推断
			KHP(row, 1) = K(row) * K(1) * variance;
			KHP(row, 2) = K(row) * K(2) * variance;
			KHP(row, 3) = K(row) * K(3) * variance;
			KHP(row, 4) = K(row) * K(4) * variance;
			KHP(row, 5) = K(row) * K(5) * variance;
			KHP(row, 6) = K(row) * K(6) * variance;
			KHP(row, 7) = K(row) * K(7) * variance;
			KHP(row, 8) = K(row) * K(8) * variance;
			KHP(row, 9) = K(row) * K(9) * variance;
			KHP(row, 10) = K(row) * K(10) * variance;
			KHP(row, 11) = K(row) * K(11) * variance;
			KHP(row, 12) = K(row) * K(12) * variance;
			KHP(row, 13) = K(row) * K(13) * variance;
			KHP(row, 14) = K(row) * K(14) * variance;
			KHP(row, 15) = K(row) * K(15) * variance;
			KHP(row, 16) = K(row) * K(16) * variance;
			KHP(row, 17) = K(row) * K(17) * variance;
			KHP(row, 18) = K(row) * K(18) * variance;
			KHP(row, 19) = K(row) * K(19) * variance;
			KHP(row, 20) = K(row) * K(20) * variance;
			KHP(row, 21) = K(row) * K(21) * variance;
			KHP(row, 22) = K(row) * K(22) * variance;
			KHP(row, 23) = K(row) * K(23) * variance;

		end

		is_healthy = checkAndFixCovarianceUpdate(KHP);      %方差大于0，否则是有问题的

		if (is_healthy) 
			% apply the covariance corrections
			P = P - KHP;

			%fixCovarianceErrors(true);

			% apply the state corrections
			fuse(K, innovation);
            assignin("base","innovation",innovation);
		end

end