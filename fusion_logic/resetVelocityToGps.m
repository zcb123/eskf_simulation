function resetVelocityToGps(gps_sample)

    global P;

    resetVelocityTo(gps_sample.vel_ned);

    P(4,4) = gps_sample.sacc^2;
    P(5,5) = gps_sample.sacc^2;
    P(6,6) = gps_sample.sacc^2;


    
end

