clear 
close all

load("data/N41_2024-11-19_11-19-29.mat");

run("init/global_init.m");

len = length(data.IMU1.t);
imu_t = double(data.IMU1.t)/1e6;
imu_dt = 0.002*ones(len,1);
imu_dt(2:end,1) = diff(imu_t);
imu_gyro = [data.IMU1.GY data.IMU1.GX -data.IMU1.GZ];
imu_acc = [data.IMU1.AY data.IMU1.AX -data.IMU1.AZ];

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

figure('Name','vehivle acc')
subplot(311)
plot(imu_t,imu_acc(:,1),vehicle_t,vehicle_acc(:,1))
grid on 
subplot(312)
plot(imu_t,imu_acc(:,2),vehicle_t,vehicle_acc(:,2))
grid on 
subplot(313)
plot(imu_t,imu_acc(:,3),vehicle_t,vehicle_acc(:,3))
grid on 
figure('Name','vehivle gyro')
subplot(311)
plot(imu_t,imu_gyro(:,1),vehicle_t,vehicle_gyro(:,1))
grid on 
subplot(312)
plot(imu_t,imu_gyro(:,2),vehicle_t,vehicle_gyro(:,2))
grid on 
subplot(313)
plot(imu_t,imu_gyro(:,3),vehicle_t,vehicle_gyro(:,3))
grid on 

clear index imu_t_update imu_gyro_update imu_acc_update imu_gyro_integral_output imu_gyro_integral_dt...
imu_gyro_integral imu_acc_integral_output imu_acc_integral_dt imu_acc_integral...
acc_dt_inv gyro_dt_inv;

clear gyro_coningIntegral acc_integral;

%%
acc_lpf_2o = LPF_2p(250,30);
gyro_lpf_2o = LPF_2p(250,20);

len_t = length(vehicle_t);
acc_filted = zeros(len_t,3);
gyro_filted = zeros(len_t,3);

for i=1:len_t
    [acc_lpf_2o,acc_filted(i,:)] = acc_lpf_2o.apply(vehicle_acc(i,:));
    [gyro_lpf_2o,gyro_filted(i,:)] = gyro_lpf_2o.apply(vehicle_gyro(i,:));

end

figure
subplot(311)
plot(vehicle_t,vehicle_acc(:,1),vehicle_t,acc_filted(:,1))
grid on
legend('aX','aXF');
subplot(312)
plot(vehicle_t,vehicle_acc(:,2),vehicle_t,acc_filted(:,2))
grid on
legend('aY','aYF');
subplot(313)
plot(vehicle_t,vehicle_acc(:,3),vehicle_t,acc_filted(:,3))
grid on
legend('aZ','aZF');

figure
subplot(311)
plot(vehicle_t,vehicle_gyro(:,1),vehicle_t,gyro_filted(:,1))
grid on
legend('gX','gXF');
subplot(312)
plot(vehicle_t,vehicle_gyro(:,2),vehicle_t,gyro_filted(:,2))
grid on
legend('gY','gYF');
subplot(313)
plot(vehicle_t,vehicle_gyro(:,3),vehicle_t,gyro_filted(:,3))
grid on
legend('gZ','gZF');
%%
global params imu_sample_delayed dt_imu_avg;
imu_sample_delayed = struct('time_us',uint64(0),...
                            'delta_ang',single([0 0 0]'),...
                            'delta_vel',single([0 0 0]'),...)
                            'delta_ang_dt',single(0),...
                            'delta_vel_dt',single(0),...
                            'delta_ang_clipping',logical([0 0 0]'),...
                            'delta_vel_clipping',logical([0 0 0]'));
global imu_down_sampler;
imu_down_sampler = ImuDownSampler(params.filter_update_interval_us);

global GNSS GPSYAW;
%update default params
params.gps_pos_noise = 0.03;
params.gps_vel_noise = 0.03;
params.accel_noise = 0.35;
params.vdist_sensor_type =GNSS;
params.fusion_mode = bitor(params.fusion_mode,GPSYAW);

global time_last_imu time_last_gps time_last_mag time_last_baro

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

vibe_metrics_display = zeros(len_t,3);
time_last_move_detect_us_display = zeros(len_t,1);
dt_imu_avg_record = zeros(len_t,1);

index = 1;
rtk_index = 1;
rtk_index_last = 1;
mag_index = 1;
mag_index_last = 1;
baro_index = 1;
baro_index_last = 1;

time_last_imu_display = zeros(len_t,1);
time_last_rtk_display = zeros(len_t,1);
time_last_mag_display = zeros(len_t,1);
time_last_bar_display = zeros(len_t,1);
for i = 1:len_t
    imu_sample_new.time_us = uint64(vehicle_t(i,1)*1e6);   %转化成微秒
    imu_sample_new.delta_ang = gyro_filted(i,:)'*vehicle_dt(i,1);
    imu_sample_new.delta_vel = acc_filted(i,:)'*vehicle_dt(i,1);
    imu_sample_new.delta_ang_dt = vehicle_dt(i,1);
    imu_sample_new.delta_vel_dt = vehicle_dt(i,1);
    imu_sample_new.delta_vel_clipping = logical([0 0 0]);
    updated = setIMUData(imu_sample_new);

    rtk_dt = data.RTK.t - vehicle_t(i,1)*1e6;
    rtk_index = find(rtk_dt<1e3,1,'last');
    if rtk_index_last ~= rtk_index
        rtk_index_last = rtk_index;
		gps_msg.time_us = data.RTK.t(rtk_index,1);
		gps_msg.lat = data.RTK.lat(rtk_index,1) * 1e7;
		gps_msg.lon = data.RTK.lon(rtk_index,1) * 1e7;
		gps_msg.alt = data.RTK.alt(rtk_index,1) * 1e3;
		gps_msg.yaw = data.RTK.hding(rtk_index,1);%data.RTK.heading;
		gps_msg.yaw_offset = params.gps_yaw_offset/57.3;
		gps_msg.fix_type = data.RTK.fix(rtk_index,1);
		gps_msg.eph = 0.01;%data.RTK.hstd;
		gps_msg.epv = 0.01;%data.RTK.vstd;
		gps_msg.sacc = 0.01;%0.5;
		
		gps_msg.vel_ned = [
			data.RTK.vN(rtk_index,1)
			data.RTK.vE(rtk_index,1)
			data.RTK.vD(rtk_index,1)
		];

		gps_msg.vel_m_s = norm(gps_msg.vel_ned);
		gps_msg.vel_ned_valid = true;%vehicle_gps_position.vel_ned_valid;
		gps_msg.nsats = data.RTK.svs(rtk_index,1);
		gps_msg.pdop = data.RTK.pdop(rtk_index,1);

        setGpsData(gps_msg);

    end


    
    mag_dt = data.MAG.t - vehicle_t(i,1)*1e6;
    mag_index = find(mag_dt<1e4,1,'last');
    if mag_index_last ~= mag_index
        mag_index_last = mag_index;
		magSample.time_us = data.MAG.t(mag_index);
        magSample.mag = [data.MAG.Y(mag_index,1)*0.003 data.MAG.X(mag_index,1)*0.003 data.MAG.Z(mag_index,1)*0.003]'; %包含坐标系旋转  

        setMagData(magSample);

    end


    baro_dt = data.BAR0.t - vehicle_t(i,1)*1e6;
    baro_index = find(baro_dt<1e4,1,'last');
    if baro_index_last ~= baro_index
        baro_index_last = baro_index;
        baro.time_us = data.BAR0.t(baro_index,1);
        baro.hgt = data.BAR0.Hight;
        setBaroData(baro);
    end

    time_last_imu_display(i,1) = time_last_imu;
    time_last_rtk_display(i,1) = time_last_gps;
    time_last_mag_display(i,1) = time_last_mag;
    time_last_bar_display(i,1) = time_last_baro;


    dt_imu_avg_record(i,1) = dt_imu_avg;
    ang_out_display(i,:) = ang_out';
    vel_out_display(i,:) = vel_out';
    quat_angle_out_display(i,:) = imu_down_sampler.delta_angle_accumulated;

    vibe_metrics_display(i,:) = vibe_metrics';
    time_last_move_detect_us_display(i,1) = time_last_move_detect_us;

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

%%
figure('Name','vibe_metrics_display')
plot(vibe_metrics_display)

figure('Name','time_last_move_detect_us_display')
plot(time_last_move_detect_us_display)
%%
figure('Name','imu time check')
plot(vehicle_t,time_last_imu_display)
figure('Name','rtk time check')
plot(vehicle_t,time_last_rtk_display)
figure('Name','mag time check')
plot(vehicle_t,time_last_mag_display)
figure('Name','baro time check')
plot(vehicle_t,time_last_bar_display)
%%
figure('Name','delta_ang')
subplot(311)
plot(vehicle_t,ang_out_display(:,1),imu_delta_t,imu_delta_ang(:,1))
grid on
subplot(312)
plot(vehicle_t,ang_out_display(:,2),imu_delta_t,imu_delta_ang(:,2))
grid on
subplot(313)
plot(vehicle_t,ang_out_display(:,3),imu_delta_t,imu_delta_ang(:,3))
grid on
%%
figure('Name','delta_vel')
subplot(311)
plot(vehicle_t,vel_out_display(:,1),imu_delta_t,imu_delta_vel(:,1))
grid on
subplot(312)
plot(vehicle_t,vel_out_display(:,2),imu_delta_t,imu_delta_vel(:,2))
grid on
subplot(313)
plot(vehicle_t,vel_out_display(:,3),imu_delta_t,imu_delta_vel(:,3))
grid on
%%
figure('Name','quat_angle_out')
subplot(211)
plot(vehicle_t,quat_angle_out_display(:,1))
subplot(212)
plot(vehicle_t,quat_angle_out_display(:,2),vehicle_t,quat_angle_out_display(:,3),vehicle_t,quat_angle_out_display(:,4))

figure('Name','imu_gyro setIMUData')
subplot(311)
plot(imu_t,imu_gyro(:,1),imu_delta_t,imu_delta_ang(:,1)./imu_ang_dt(:,1));
subplot(312)
plot(imu_t,imu_gyro(:,2),imu_delta_t,imu_delta_ang(:,2)./imu_ang_dt(:,1));
subplot(313)
plot(imu_t,imu_gyro(:,3),imu_delta_t,imu_delta_ang(:,3)./imu_ang_dt(:,1));

figure('Name','imu_acc setIMUData')
subplot(311)
plot(imu_t,imu_acc(:,1),imu_delta_t,imu_delta_vel(:,1)./imu_vel_dt(:,1));
subplot(312)
plot(imu_t,imu_acc(:,2),imu_delta_t,imu_delta_vel(:,2)./imu_vel_dt(:,1));
subplot(313)
plot(imu_t,imu_acc(:,3),imu_delta_t,imu_delta_vel(:,3)./imu_vel_dt(:,1));

%%
clear time_last_bar_display time_last_mag_display time_last_rtk_display time_last_imu_display ...
    vibe_metrics_display time_last_move_detect_us_display ...
    ang_out_display vel_out_display quat_angle_out_display ...
    imu_sample_delayed_ang imu_sample_delayed_vel imu_sample_delayed_t imu_sample_ang_dt imu_sample_vel_dt...