function res = project(lat,lon)

    global CONSTANTS_RADIUS_OF_EARTH;
	lat_rad = radians(lat);
	lon_rad = radians(lon);

	sin_lat = sin(lat_rad);
	cos_lat = cos(lat_rad);

	cos_d_lon = cos(lon_rad - ref_lon);

	arg = saturation(ref_sin_lat * sin_lat + ref_cos_lat * cos_lat * cos_d_lon, -1.0,  1.0);
	c = acos(arg);

	k = 1.0;

	if fabs(c) > 0 
		k = (c / sin(c));
    end

	x = (k * (ref_cos_lat * sin_lat - ref_sin_lat * cos_lat * cos_d_lon) * CONSTANTS_RADIUS_OF_EARTH);
	y = (k * cos_lat * sin(lon_rad - ref_lon) * CONSTANTS_RADIUS_OF_EARTH);

    res = [x y]';
end