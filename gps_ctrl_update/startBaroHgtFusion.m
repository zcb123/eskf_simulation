function startBaroHgtFusion()

    global control_status ; 
    global hgt_sensor_offset;
    if ~control_status.flags.baro_ght
        if ~control_status.flags.rng_hgt
            resetHeightToBaro();
        end
        setControlBaroHeight();
        hgt_sensor_offset = 0;
    end


end