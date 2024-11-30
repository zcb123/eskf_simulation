function controlHeightFusion()

    
    
    if control_status.flags.gps_hgt && gps_hgt_accurate
        fuseGpsHgt()
    end

end


