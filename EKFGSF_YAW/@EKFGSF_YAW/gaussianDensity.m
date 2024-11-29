function res = gaussianDensity(obj,model_index)

	% calculate transpose(innovation) * inv(S) * innovation
	normDist = obj.ekf_gsf(model_index,1).innov' * (obj.ekf_gsf(model_index,1).S_inverse * obj.ekf_gsf(model_index,1).innov);
    %0.159154943 = _m_2pi_inv;
    assignin("base","normDist",normDist);
	res = 0.159154943 * sqrt(obj.ekf_gsf(model_index,1).S_det_inverse) * exp(-0.5 * normDist);

end