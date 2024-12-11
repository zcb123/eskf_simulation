function res = get_mag_strength_gauss(lat,lon)
    
    global strength_table;
    res = get_table_data(lat, lon, strength_table) * 1e-4;


end