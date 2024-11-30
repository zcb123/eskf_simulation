function controlHeightFusion()

    
    global control_status gps_hgt_accurate;

    if control_status.flags.gps_hgt && gps_hgt_accurate
        fuseGpsHgt()
    end

end


