function res = get_table_data(lat,lon,table)

    
    SAMPLING_RES = 10;
    SAMPLING_MIN_LAT = -90;
    SAMPLING_MAX_LAT = 90;
    SAMPLING_MIN_LON = -180;
    SAMPLING_MAX_LON = 180;

    lat = saturation(lat, SAMPLING_MIN_LAT, SAMPLING_MAX_LAT);

	if (lon > SAMPLING_MAX_LON) 
		lon = lon - 360;
	end

	if (lon < SAMPLING_MIN_LON) 
		lon = lon + 360;
	end

	
	min_lat = double(floor(lat / SAMPLING_RES)) * SAMPLING_RES;
	min_lon = double(floor(lon / SAMPLING_RES)) * SAMPLING_RES;

	
	[min_lat_index,min_lat] = get_lookup_table_index(min_lat, SAMPLING_MIN_LAT, SAMPLING_MAX_LAT);
	[min_lon_index,min_lon] = get_lookup_table_index(min_lon, SAMPLING_MIN_LON, SAMPLING_MAX_LON);

	data_sw = table(min_lat_index,min_lon_index);
	data_se = table(min_lat_index,min_lon_index + 1);
	data_ne = table(min_lat_index + 1,min_lon_index + 1);
	data_nw = table(min_lat_index + 1,min_lon_index);

	
	lat_scale = saturation((lat - min_lat) / SAMPLING_RES, 0, 1);
	lon_scale = saturation((lon - min_lon) / SAMPLING_RES, 0, 1);

	data_min = lon_scale * (data_se - data_sw) + data_sw;
	data_max = lon_scale * (data_ne - data_nw) + data_nw;

	res = lat_scale * (data_max - data_min) + data_min;
end