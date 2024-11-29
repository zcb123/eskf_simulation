function resetHorizontalPositionToGps(gps_sample)

    global P;
    
    resetHorizontalPositionTo(gps_sample.pos_ned)


    P(7,7) = gps_sample_delayed.hdop*gps_sample_delayed.hdop;
    P(8,8) = P(7,7);


end

