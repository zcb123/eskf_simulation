function resetHorizontalPositionToGps(gps_sample)

    global P;
    
    resetHorizontalPositionTo(gps_sample.pos(1:2,1))


    P(7,7) = gps_sample.hacc*gps_sample.hacc;
    P(8,8) = P(7,7);


end

