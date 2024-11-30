function clearMagCov()

    global P params;
    global mag_decl_cov_reset;
    P(16,16) = 0;
    P(17,17) = 0;
    P(18,18) = 0;
    P(19,19) = 0;
    P(20,20) = 0;
    P(21,21) = 0;
    mag_decl_cov_reset = false;
    
    

end

