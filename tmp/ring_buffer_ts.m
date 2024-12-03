clear
close all
load("data/N41_2024-11-19_11-19-29.mat");
%%
gps_buffer = ring_buffer(3);
gps_len = length(data.RTK.t);

for gps_index = 1:gps_len

    gps_sample_delayed.time_us = data.RTK.t(gps_index,1);
    gps_sample_delayed.lon = data.RTK.lon(gps_index,1);
    gps_sample_delayed.lat = data.RTK.lat(gps_index,1);
    gps_sample_delayed.alt = data.RTK.alt(gps_index,1);
    gps_sample_delayed.pos_ned = [data.RTK.pN(gps_index,1) data.RTK.pE(gps_index,1) data.RTK.pD(gps_index,1)]';
    gps_sample_delayed.vel_ned = [data.RTK.vN(gps_index,1) data.RTK.vE(gps_index,1) data.RTK.vD(gps_index,1)]';
    gps_sample_delayed.yaw = data.RTK.hding(gps_index,1);      
    gps_sample_delayed.hgt = data.RTK.alt(gps_index,1)*1e-3;
    gps_sample_delayed.hacc = data.RTK.hdop(gps_index,1);        %这个赋值有待商榷
    gps_sample_delayed.pdop = data.RTK.pdop(gps_index,1);
    gps_sample_delayed.hdop = data.RTK.hdop(gps_index,1);
    gps_sample_delayed.sacc = 0.5;
    gps_sample_delayed.vacc = 0.01;         %代码中0.01
    gps_buffer.push(gps_sample_delayed);
    if mod(gps_index,3) == 0
        gps_data = gps_buffer.pop_first_older_than();
%     if gps_buffer
%         disp('dfa');
%     end
    end
end
len_imu = length(vehicle_t);
for i = 1:len_imu 

    imu_sample_new.time_us = uint64(vehicle_t(i,1)*1e6);   %转化成微秒
    imu_sample_new.delta_ang = gyro_filted(i,:)'*vehicle_dt(i,1);
    imu_sample_new.delta_vel = acc_filted(i,:)'*vehicle_dt(i,1);
    imu_sample_new.delta_ang_dt = vehicle_dt(i,1);
    imu_sample_new.delta_vel_dt = vehicle_dt(i,1);
    imu_sample_new.delta_vel_clipping = logical([0 0 0]);
    

end