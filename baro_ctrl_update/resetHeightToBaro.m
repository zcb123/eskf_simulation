function resetHeightToBaro()
 
    global P baro_sample_delayed baro_hgt_offset params;
    resetVerticalPositionTo(-baro_sample_delayed.hgt + baro_hgt_offset);
    P(9,9) = params.baro_noise;

end

