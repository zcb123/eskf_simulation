function resetQuatCov()
    global P params;

    P(1,1) = 0;
    P(2,2) = 0;
    P(3,3) = 0;

    P(1,1) = params.initial_tilt_err;
    P(2,2) = params.initial_tilt_err;
    P(3,3) = params.initial_tilt_err;
    
end