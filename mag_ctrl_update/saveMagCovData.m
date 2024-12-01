function saveMagCovData()
    global P;
    global saved_mag_bf_variance saved_mag_ef_ne_covmat saved_mag_ef_d_variance;
    
    saved_mag_bf_variance = [P(19,19) P(20,20) P(21,21)];
    saved_mag_ef_ne_covmat = P(16:17,16:17);
    saved_mag_ef_d_variance = P(18,18);

end

