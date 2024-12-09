function res = calcEarthRateNED(lat_rad)
    
    global CONSTANTS_EARTH_SPIN_RATE;
    CONSTANTS_EARTH_SPIN_RATE = 7.2921150e-5;

     res = [CONSTANTS_EARTH_SPIN_RATE * cos(lat_rad),...
			0,...
			-CONSTANTS_EARTH_SPIN_RATE * sin(lat_rad)];

end

