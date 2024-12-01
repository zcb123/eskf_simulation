function uncorrelateCovarianceSetVariance(width,first,vec)

    global P;
    P(first:first+width,first:first+width) = 0;

    vec_idx = 0;
	for idx = first:first + Width
		P(idx, idx) = vec(vec_idx);
		vec_idx = vec_idx + 1;
    end


end

