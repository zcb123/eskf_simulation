function res = get_mag_inclination_radians(lat,lon)
    global inclination_table;
    res = get_table_data(lat, lon, inclination_table) * 1e-4;;
end