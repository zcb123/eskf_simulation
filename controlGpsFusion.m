function controlGpsFusion(gps_data,imu_t,params)
    persistent gps_index_last;
    if isempty(gps_index_last)
        gps_index_last = 0;
    end

    dt = gps_data.t - imu_t;
    gps_index = find(dt<1e3,1,'last');
    assignin("base","gps_index",gps_index);
    if gps_index_last~=gps_index    %目前都默认gps数据是能用的
        gps_sample_delayed.time_us = gps_data.t(gps_index,1);
        gps_sample_delayed.pos_ned = [gps_data.pN(gps_index,1) gps_data.pE(gps_index,1) gps_data.pD(gps_index,1)];
        gps_sample_delayed.vel_ned = [gps_data.vN(gps_index,1) gps_data.vE(gps_index,1) gps_data.vD(gps_index,1)];
        gps_sample_delayed.yaw = gps_data.hding(gps_index,1);
        gps_sample_delayed.hgt = gps_data.alt(gps_index,1)*1e-3;
        gps_sample_delayed.hacc = gps_data.hdop(gps_index,1);        %这个赋值有待商榷
        gps_sample_delayed.sacc = 0.5;
        gps_sample_delayed.fix_type = gps_data.fix(gps_index,1);
        fuseGpsVelPos(gps_sample_delayed,params);
        gps_index_last = gps_index;
    end



    
end