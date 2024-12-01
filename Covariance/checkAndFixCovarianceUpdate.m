function ret = checkAndFixCovarianceUpdate(KHP)

    global P;
    healthy = true;

	for i = 1:23
		if (P(i, i) < KHP(i, i)) 
            %zero rows and columns
			P(i,:) = 0;
            P(:,i) = 0;
            P(i,i) = 0;
			healthy = false;
		end
	end

	ret = healthy;

end
