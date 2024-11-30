function startBaroHgtFusion()

    global control_status params P;

    if ~control_status.flags.baro_ght
        if ~control_status.flags.rng_hgt
            P(9,9) = params.baro_noise;
        end
    end


end