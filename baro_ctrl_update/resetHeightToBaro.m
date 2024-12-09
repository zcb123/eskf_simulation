function resetHeightToBaro()
 
    global baro_sample_delayed baro_hgt_offset params;
    resetVerticalPositionTo(-baro_sample_delayed.hgt + baro_hgt_offset);
    uncorrelateCovarianceSetVariance(1,9,params.baro_noise)
    

end

