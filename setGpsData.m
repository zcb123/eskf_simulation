function  setGpsData(gps_data,gps_index)

global params;
global gps_sample_delayed imu_sample_delayed;
global gps_hgt_accurate;

    gps_sample_delayed.time_us = gps_data.t(gps_index,1);
    gps_sample_delayed.lon = gps_data.lon(gps_index,1);
    gps_sample_delayed.lat = gps_data.lat(gps_index,1);
    gps_sample_delayed.alt = gps_data.alt(gps_index,1);
    gps_sample_delayed.pos_ned = [gps_data.pN(gps_index,1) gps_data.pE(gps_index,1) gps_data.pD(gps_index,1)]';
    gps_sample_delayed.vel_ned = [gps_data.vN(gps_index,1) gps_data.vE(gps_index,1) gps_data.vD(gps_index,1)]';
    gps_sample_delayed.yaw = gps_data.hding(gps_index,1);      
    gps_sample_delayed.hgt = gps_data.alt(gps_index,1)*1e-3;
    gps_sample_delayed.hacc = gps_data.hdop(gps_index,1);        %这个赋值有待商榷
    gps_sample_delayed.pdop = gps_data.pdop(gps_index,1);
    gps_sample_delayed.hdop = gps_data.hdop(gps_index,1);
    gps_sample_delayed.sacc = 0.5;
    gps_sample_delayed.vacc = 0.01;         %代码中0.01

    gps_sample_delayed.fix_type = gps_data.fix(gps_index,1);

    gps_hgt_accurate = (gps_sample_delayed.vacc < params.req_vacc) && (gps_sample_delayed.fix_type == 6);



    collect_gps(gps_sample_delayed);
    
 

    
end
