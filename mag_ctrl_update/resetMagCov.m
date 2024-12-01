function resetMagCov()
    
    global P params control_status;
    global mag_decl_cov_reset;
    P(16,16) = 0;
    P(17,17) = 0;
    P(18,18) = 0;
    P(19,19) = 0;
    P(20,20) = 0;
    P(21,21) = 0;
    mag_decl_cov_reset = false;
    
    P(16,16) = sq(params.mag_noise);
    P(17,17) = sq(params.mag_noise);
    P(18,18) = sq(params.mag_noise);
    P(19,19) = sq(params.mag_noise);
    P(20,20) = sq(params.mag_noise);
    P(21,21) = sq(params.mag_noise);


    if ~control_status.flags.mag_3D
        saveMagCovData();
    end

end

