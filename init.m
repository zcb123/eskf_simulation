clear 
% foldername = 'C:\Users\89565\Desktop\自研飞控\TestData\十一月数据\1119\静态试验\';
% filename = 'N41_2024-11-19_11-19-29.log';
% data = Parse_Data(foldername,filename);
load("data/N41_2024-11-19_11-19-29.mat");
%%
clear states imu_sample_delayed dt_imu_avg  dt_ekf_avg P;   %清理旧的变量

global states imu_sample_delayed dt_imu_avg  dt_ekf_avg P P_M;
states = struct('quat_nominal',single([1 0 0 0]'),...
                        'vel',single([0 0 0]'),...
                        'pos',single([0 0 0]'),...
                        'delta_ang_bias',single([0 0 0]'),...
                        'delta_vel_bias',single([0 0 0]'),...
                        'mag_I',single([0 0 0]'),...
                        'mag_B',single([0 0 0]'),...
                        'wind_vel',single([0 0]'));

imu_sample_delayed = struct('time_us',uint64(0),...
                            'delta_ang',single([0 0 0]'),...
                            'delta_vel',single([0 0 0]'),...)
                            'delta_ang_dt',single(0),...
                            'delta_vel_dt',single(0),...
                            'delta_ang_clipping',logical([0 0 0]'),...
                            'delta_vel_clipping',logical([0 0 0]'));

P = zeros(23,23);
P_M = zeros(23,23);

dt_imu_avg = single(0.002);
dt_ekf_avg = single(0.008);

CONSTANTS_ONE_G = single(9.80665);


imu.time_us = uint64(0);
imu.delta_ang = single([0 0 0]');
imu.delta_vel = single([1 0 0]');
imu.delta_ang_dt = single(0);
imu.delta_vel_dt = single(0);
imu.delta_ang_clipping = logical([0 0 0]');
imu.delta_vel_clipping = logical([0 0 0]');


len = length(data.IMU1.t);
imu_t = double(data.IMU1.t)/1e6;
imu_dt = 0.002*ones(len,1);
imu_dt(2:end,1) = diff(imu_t);
imu_gyro = [data.IMU1.GY data.IMU1.GX -data.IMU1.GZ];
imu_acc = [data.IMU1.AY data.IMU1.AX -data.IMU1.AZ];
% imu_delta_ang = imu_gyro.*imu_dt;
% %imu_dt太不均匀以至于delta_ang的变化被imu_dt的变化淹没了
% %imu_gyro表现出imu_dt的变化趋势;
% %imu_acc也是同理
% imu_delta_vel = imu_acc.*imu_dt;
imu_delta_ang = imu_gyro*0.002;
imu_delta_vel = imu_acc*0.002;
correct_update = logical(true);
quat_new = single(zeros(len,4));
vel_new = single(zeros(len,3));
pos_new = single(zeros(len,3));

%%  传感器数据检查
% figure('Name','imu_gyro')
% subplot(311)
% plot(imu_t,imu_gyro(:,1));
% subplot(312)
% plot(imu_t,imu_gyro(:,2));
% subplot(313)
% plot(imu_t,imu_gyro(:,3));
%%
% figure('Name','delta_ang')
% subplot(311)
% plot(imu_t,imu_delta_ang(:,1));
% subplot(312)
% plot(imu_t,imu_delta_ang(:,2));
% subplot(313)
% plot(imu_t,imu_delta_ang(:,3));
%%
% figure('Name','imu_acc')
% subplot(311)
% plot(imu_t,imu_acc(:,1));
% subplot(312)
% plot(imu_t,imu_acc(:,2));
% subplot(313)
% plot(imu_t,imu_acc(:,3));
% 
% figure('Name','delta_vel')
% subplot(311)
% plot(imu_t,imu_delta_vel(:,1));
% subplot(312)
% plot(imu_t,imu_delta_vel(:,2));
% subplot(313)
% plot(imu_t,imu_delta_vel(:,3));
%%
% figure
% plot(imu_dt)
%%
params.imu_pos_body = single([1 0 0]');
params.gps_pos_body = single([0 0 0]');
params.vel_Tau = single(0.25);
params.pos_Tau = single(0.25);
params.filter_update_interval_us = single(8000);
params.gyro_noise = single(1.5e-2);
params.gyro_bias_p_noise = single(1e-3);
params.accel_noise = single(3.5e-1);
params.accel_bias_p_noise = single(3e-3);
params.wind_vel_p_noise = single(1e-1);
params.wind_vel_p_noise_scaler = single(0.5);
params.acc_bias_learn_tc = single(0.5);
params.acc_bias_learn_gyr_lim = single(3);
params.acc_bias_learn_acc_lim = single(25);
params.fusion_mode = uint8(1);
params.mage_p_noise = single(1e-3);
params.magb_p_noise = single(1e-4);
params.initial_wind_uncertainty = single(1);
params.pos_noaid_noise = single(10);
params.gps_pos_innov_gate = single(5);
params.gps_vel_innov_gate = single(5);
params.gps_pos_noise = single(0.5);

control_status.flags.mag_3D = logical(true);
control_status.flags.wind = logical(false);

fault_status.flags.bad_vel_N = logical(true);