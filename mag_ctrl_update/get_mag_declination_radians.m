function res = get_mag_declination_radians(lat, lon)

    global declination_table;
    res = get_table_data(lat,lon, declination_table) * 1e-4;% declination table stored as 10^-4 radians

end

