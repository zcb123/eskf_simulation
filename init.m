global states dt_imu_avg  dt_ekf_avg ;
states = struct('quat_nominal',single([1 0 0 0]'),...
                        'vel',single([0 0 0]'),...
                        'pos',single([0 0 0]'),...
                        'delta_ang_bias',single([0 0 0]'),...
                        'delta_vel_bias',single([0 0 0]'),...
                        'mag_I',single([0 0 0]'),...
                        'mag_B',single([0 0 0]'),...
                        'wind_vel',single([0 0]'));
dt_imu_avg = single(1);
dt_ekf_avg = single(1);

CONSTANTS_ONE_G = single(9.80665);

imu.time_us = uint64(0);
imu.delta_ang = single([0 0 0]');
imu.delta_vel = single([1 0 0]');
imu.delta_ang_dt = single(0);
imu.delta_vel_dt = single(0);
imu_sample_delayed = imu;
params.imu_pos_body = single([1 0 0]');
params.vel_Tau = single(0.25);
params.pos_Tau = single(0.25);
params.filter_update_interval_us = single(8000);

len = length(data.IMU1.t);
imu_t = double(data.IMU1.t)/1e6;
imu_dt = 0.002*ones(len,1);
imu_dt(2:end,1) = diff(imu_t);
imu_gyro = [data.IMU1.GY data.IMU1.GX -data.IMU1.GZ];
imu_acc = [data.IMU1.AY data.IMU1.AX -data.IMU1.AZ];
imu_delta_ang = imu_gyro.*imu_dt;
imu_delta_vel = imu_acc.*imu_dt;
correct_update = logical(true);
quat_new = single(zeros(len,4));
vel_new = single(zeros(len,3));
pos_new = single(zeros(len,3));

