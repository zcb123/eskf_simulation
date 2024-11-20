function setIMUData(imu)

global dt_imu_avg;
persistent time_us_last;
if isempty(time_us_last)
    time_us_last = uint64(0);
end

dt = double(imu.time_us - time_us_last)/1e6;
dt = saturation(dt,1e-4,0.02);

time_us_last = imu.time_us;

if(time_us_last>0)
    dt_imu_avg = 0.8*dt_imu_avg+0.2*dt;
end


end