function uncorrelateCovarianceSetVariance(width,first,vec)

    global P;
    P(first:first+width-1,first:first+width-1) = 0;

    vec_idx = 0;
	for idx = first:first + width-1
        vec_idx = vec_idx + 1;
		P(idx, idx) = vec(vec_idx);
		
    end


end

