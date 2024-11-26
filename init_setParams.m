clear 
close all
load("data/N41_2024-11-19_11-19-29.mat");

clear states imu_sample_delayed dt_imu_avg  dt_ekf_avg P P_M;   %清理旧的变量

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
dt_imu_avg = 0.004;
dt_ekf_avg = 0.008;

CONSTANTS_ONE_G = single(9.80665);

len = length(data.IMU1.t);
imu_t = double(data.IMU1.t)/1e6;
imu_dt = 0.002*ones(len,1);
imu_dt(2:end,1) = diff(imu_t);
imu_gyro = [data.IMU1.GY data.IMU1.GX -data.IMU1.GZ];
imu_acc = [data.IMU1.AY data.IMU1.AX -data.IMU1.AZ];


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
%%  参数初始化
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
params.gps_vel_noise = single(0.3);
params.switch_on_gyro_bias = single(0.1);
params.switch_on_accel_bias = single(0.2);
params.mag_noise = single(5e-2);
params.initial_wind_uncertainty = single(1);
params.initial_tilt_err = single(0.1);

control_status.flags.mag_3D = logical(true);
control_status.flags.wind = logical(false);

fault_status.flags.bad_vel_N = logical(true);
%% 变量初始化
P(1,1) = params.initial_tilt_err^2;
P(2,2) = P(1,1);
P(3,3) = P(1,1);
P(4,4) = params.gps_vel_noise;
P(5,5) = P(4,4);
P(6,6) = P(4,4);
P(7,7) = params.gps_pos_noise;
P(8,8) = P(7,7);
P(9,9) = data.RTK.pdop(1,1)^2 - data.RTK.hdop(1,1)^2;                %等于gps垂直精度(vacc)的平方
P(10,10) = params.switch_on_gyro_bias*dt_ekf_avg;
P(11,11) = P(10,10);
P(12,12) = P(10,10);
P(13,13) = params.switch_on_accel_bias*dt_ekf_avg;
P(14,14) = P(13,13);
P(15,15) = P(13,13);

P(16,16) = params.mag_noise^2;
P(17,17) = P(16,16);
P(18,18) = P(16,16);
P(19,19) = params.mag_noise^2;
P(20,20) = P(19,19);
P(21,21) = P(19,19);
P(22,22) = params.initial_wind_uncertainty;
P(23,23) = P(22,22);

P_M = P;


%%
clear yawEstimator;
yawEstimator = EKFGSF_YAW();
%%
% fs = 500;
% fc = 30;
% order = 2;
% Wn = fc/(fs/2);
% [b,a] = butter(order,Wn,'low');

acc_lpf = LPF_2p(250,30);
gyro_lpf = LPF_2p(250,20);