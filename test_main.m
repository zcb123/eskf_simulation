% run('flight_log_import.m');
% run("bus\Geskf_Res.m");

% 

clear 
foldername = 'C:\Users\89565\Desktop\自研飞控\TestData\十月份数据\1030\';
filename = 'N447_2024-10-30_17-45-26.log';
data = Parse_Data(foldername,filename);

imu_t = double(data.IMU0.t)/1e6;
imu_len = length(imu_t);
imu_dt = 0.002*ones(imu_len,1);
imu_dt(1:end-1,1) = diff(imu_t);
AX = data.IMU0.AX;
AY = data.IMU0.AY;
AZ = data.IMU0.AZ;

states.quat_nominal = single([1 0 0 0]);
states.vel = single([0 0 0]);
states.pos = single([0 0 0]);
states.delta_ang_bias = single([0 0 0]);
states.delta_vel_bias = single([0 0 0]);


