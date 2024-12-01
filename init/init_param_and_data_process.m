%% 
clear 
close all

load("data/N41_2024-11-19_11-19-29.mat");

len = length(data.IMU1.t);
imu_t = double(data.IMU1.t)/1e6;
imu_dt = 0.002*ones(len,1);
imu_dt(2:end,1) = diff(imu_t);
imu_gyro = [data.IMU1.GY data.IMU1.GX -data.IMU1.GZ];
imu_acc = [data.IMU1.AY data.IMU1.AX -data.IMU1.AZ];
%% 
global states dt_imu_avg  dt_ekf_avg ;
states = struct('quat_nominal',single([1 0 0 0]'),...
                        'vel',single([0 0 0]'),...
                        'pos',single([0 0 0]'),...
                        'delta_ang_bias',single([0 0 0]'),...
                        'delta_vel_bias',single([0 0 0]'),...
                        'mag_I',single([0 0 0]'),...
                        'mag_B',single([0 0 0]'),...
                        'wind_vel',single([0 0]'));
dt_imu_avg = 0.004;
dt_ekf_avg = 0.008;
% imu_sample_delayed = struct('time_us',uint64(0),...
%                             'delta_ang',single([0 0 0]'),...
%                             'delta_vel',single([0 0 0]'),...)
%                             'delta_ang_dt',single(0),...
%                             'delta_vel_dt',single(0),...
%                             'delta_ang_clipping',logical([0 0 0]'),...
%                             'delta_vel_clipping',logical([0 0 0]'));

global output_new output_buffer head_index tail_index;
head_index = 1;
tail_index = 1;
output_new = struct('time_us',uint64(0),'quat_nominal',single([1 0 0 0]'),'vel',single([0 0 0]'),'pos',single([0 0 0]'));

output_buffer = [output_new;
                 output_new;
                 output_new;];


global CONSTANTS_ONE_G FLT_EPSILON;
CONSTANTS_ONE_G = single(9.80665);
FLT_EPSILON = 1.192e-7;


global R_to_earth;
R_to_earth = zeros(3,3);

global time_last_gps_yaw_fuse time_last_imu;

time_last_imu = 0;
time_last_gps_yaw_fuse = 0;

global yaw_delta_ef;
yaw_delta_ef = 0;

%% 参数初始化

global params control_status fault_status;

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
params.gps_yaw_offset = single(180);                                %出货机天线航向偏置180，碳管机偏置90
params.gps_heading_noise = single(0.1);
params.heading_innov_gate = single(2.6);
params.reset_timeout_max = 7000000;         %微秒
params.EKFGSF_reset_delay = 1000000;
params.EKFGSF_yaw_err_max = 0.262;
params.mag_fusion_type = 0; %AUTO=0;HEADING=1,MAG_3D=2,UNUSED=3,INDOOR=4,NONE=5
params.req_vacc = 8.0;
params.baro_noise = 3.5;
params.EKFGSF_reset_count_limit = 3;
params.mag_declination_deg = 0;
params.mag_acc_gate = 0.5;
params.mag_yaw_rate_gate = 0.25;
params.check_mag_strength = 0;
params.mag_declination_source = 7;
params.mag_heading_noise = 3e-1;

control_status.flags.mag_3D = logical(true);
control_status.flags.wind = logical(false);
control_status.flags.in_air = logical(false);
control_status.flags.tilt_align = false;
control_status.flags.yaw_align = false;
control_status.flags.gps = logical(true);
control_status.flags.gps_yaw = false;
control_status.flags.gps_yaw_fault = false;
control_status.flags.gps_hgt = true;
control_status.flags.baro_ght = false;
control_status.flags.rng_hgt = false;
control_status.flags.mag_field_disturbed = false;
control_status.flags.vehicle_at_rest = true;
control_status.flags.mag_aligned_in_flight = false;


fault_status.flags.bad_vel_N = logical(true);
fault_status.flags.bad_hdg = false;

%% 变量初始化
global P P_M;
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

clear yawEstimator;
global yawEstimator;
yawEstimator = EKFGSF_YAW();


acc_lpf = LPF_2p(250,30);
gyro_lpf = LPF_2p(250,20);

%% 传感器数据预处理  梯形积分与锥运动补偿
imu_acc_integral = zeros(len,3);
imu_acc_integral_dt = zeros(len,1);
imu_acc_integral_output = NaN(len,3);
imu_gyro_integral = zeros(len,3);
imu_gyro_integral_dt = zeros(len,1);
imu_gyro_integral_output = NaN(len,3);
imu_acc_update = NaN(len,3);
imu_gyro_update = NaN(len,3);
imu_t_update = NaN(len,1);
index = 1;
for i = 1:len

    [imu_acc_integral(i,:),imu_acc_integral_dt(i,1),acc_flag] = acc_integral(imu_acc(i,:),imu_dt(i,1));
    acc_dt_inv = 1e6/imu_acc_integral_dt(i,1);
    imu_acc_integral_output(i,:) = imu_acc_integral(i,:)*acc_dt_inv;

    [imu_gyro_integral(i,:),imu_gyro_integral_dt(i,1),gyro_flag] = gyro_coningIntegral(imu_gyro(i,:),imu_dt(i,1));
    gyro_dt_inv = 1e6/imu_gyro_integral_dt(i,1);
    imu_gyro_integral_output(i,:) = imu_gyro_integral(i,:)*gyro_dt_inv;

    if acc_flag && gyro_flag
        imu_acc_update(index,:) = imu_acc_integral_output(i,:);
        imu_gyro_update(index,:) = imu_gyro_integral_output(i,:);
        imu_t_update(index,:) = imu_t(i,1);                         %两个时间相等，任意提取一个即可
        index = index + 1;
    end

end
%将非NaN元素提取出来
vehicle_acc = getNonNaN(imu_acc_update,3);
vehicle_gyro = getNonNaN(imu_gyro_update,3);
vehicle_t = getNonNaN(imu_t_update,1);

clear index imu_t_update imu_gyro_update imu_acc_update imu_gyro_integral_output imu_gyro_integral_dt...
imu_gyro_integral imu_acc_integral_output imu_acc_integral_dt imu_acc_integral...
acc_dt_inv gyro_dt_inv;

clear gyro_coningIntegral acc_integral;

%% 
len_t = length(vehicle_t);
acc_filted = zeros(len_t,3);
gyro_filted = zeros(len_t,3);

for i=1:len_t
    [acc_lpf,acc_filted(i,:)] = acc_lpf.apply(vehicle_acc(i,:));
    [gyro_lpf,gyro_filted(i,:)] = gyro_lpf.apply(vehicle_gyro(i,:));

end

%%

global imu_sample_delayed;
imu_sample_delayed = struct('time_us',uint64(0),...
                            'delta_ang',single([0 0 0]'),...
                            'delta_vel',single([0 0 0]'),...)
                            'delta_ang_dt',single(0),...
                            'delta_vel_dt',single(0),...
                            'delta_ang_clipping',logical([0 0 0]'),...
                            'delta_vel_clipping',logical([0 0 0]'));

reset_setIMUData(0.0025);

len_t = length(vehicle_t);
vehicle_dt = zeros(len_t,1);
vehicle_dt(end,1) = 0.0025;
vehicle_dt(1:end-1,:) = diff(vehicle_t);

ang_out_display = single(zeros(len_t,3));
vel_out_display = single(zeros(len_t,3));
quat_angle_out_display = single(zeros(len_t,4));



imu_sample_delta_ang = NaN(len_t,3);
imu_sample_delta_vel = NaN(len_t,3);

imu_sample_delayed_ang = NaN(len_t,3);
imu_sample_delayed_vel = NaN(len_t,3);
imu_sample_delayed_t = NaN(len_t,1);
imu_sample_ang_dt = NaN(len_t,1);
imu_sample_vel_dt = NaN(len_t,1);

dt_imu_avg_record = zeros(len_t,1);
imu_sample_last_imu_t = NaN(len_t,1);
index = 1;
for i = 1:len_t

    imu_sample_new.time_us = uint64(vehicle_t(i,1)*1e6);   %转化成微秒
    imu_sample_new.delta_ang = gyro_filted(i,:)'*vehicle_dt(i,1);
    imu_sample_new.delta_vel = acc_filted(i,:)'*vehicle_dt(i,1);
    imu_sample_new.delta_ang_dt = vehicle_dt(i,1);
    imu_sample_new.delta_vel_dt = vehicle_dt(i,1);
    imu_sample_new.delta_vel_clipping = logical([0 0 0]);
    
    imu_sample_delta_ang(i,:) = imu_sample_new.delta_ang';
    imu_sample_delta_vel(i,:) = imu_sample_new.delta_vel';

    
    updated = setIMUData(imu_sample_new);
    

    imu_sample_last_imu_t(i,1) = time_last_imu;


    dt_imu_avg_record(i,1) = dt_imu_avg;
    ang_out_display(i,:) = ang_out';
    vel_out_display(i,:) = vel_out';
    quat_angle_out_display(i,:) = quat_angle_out;
    
    if updated     
       imu_sample_delayed_ang(index,:) = imu_sample_delayed.delta_ang';
       imu_sample_delayed_vel(index,:) = imu_sample_delayed.delta_vel';
       imu_sample_delayed_t(index,1) = vehicle_t(i,1);
       imu_sample_ang_dt(index,1) = imu_sample_delayed.delta_ang_dt;
       imu_sample_vel_dt(index,1) = imu_sample_delayed.delta_vel_dt;
       index = index + 1;

    end

end


imu_delta_ang = getNonNaN(imu_sample_delayed_ang,3);
imu_delta_vel = getNonNaN(imu_sample_delayed_vel,3);
imu_delta_t = getNonNaN(imu_sample_delayed_t,1);
imu_ang_dt = getNonNaN(imu_sample_ang_dt,1);
imu_vel_dt = getNonNaN(imu_sample_vel_dt,1);

clear delta_ang_dt_avg target_dt_s required_samples target_dt_s min_dt_s...
    imu_sample_delta_ang  imu_sample_delta_vel quat_angle_out ...
    ang_out_display vel_out_display quat_angle_out_display ...
    imu_sample_delayed_t imu_sample_delayed_ang imu_sample_delayed_vel ...
    imu_sample_ang_dt imu_sample_vel_dt ang_out vel_out;


clear imu_sample_delayed imu_sample_new index len len_t;

clear setIMUData;


