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
acc_lpf = LPF_2p(250,30);
gyro_lpf = LPF_2p(250,20);

len_t = length(vehicle_t);
acc_filted = zeros(len_t,3);
gyro_filted = zeros(len_t,3);

for i=1:len_t
    [acc_lpf,acc_filted(i,:)] = acc_lpf.apply(vehicle_acc(i,:));
    [gyro_lpf,gyro_filted(i,:)] = gyro_lpf.apply(vehicle_gyro(i,:));

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
global params imu_sample_delayed;
imu_sample_delayed = struct('time_us',uint64(0),...
                            'delta_ang',single([0 0 0]'),...
                            'delta_vel',single([0 0 0]'),...)
                            'delta_ang_dt',single(0),...
                            'delta_vel_dt',single(0),...
                            'delta_ang_clipping',logical([0 0 0]'),...
                            'delta_vel_clipping',logical([0 0 0]'));
global imu_down_sampler;
imu_down_sampler = ImuDownSampler(params.filter_update_interval_us);

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
index = 1;
for i = 1:len_t
    imu_sample_new.time_us = uint64(vehicle_t(i,1)*1e6);   %转化成微秒
    imu_sample_new.delta_ang = gyro_filted(i,:)'*vehicle_dt(i,1);
    imu_sample_new.delta_vel = acc_filted(i,:)'*vehicle_dt(i,1);
    imu_sample_new.delta_ang_dt = vehicle_dt(i,1);
    imu_sample_new.delta_vel_dt = vehicle_dt(i,1);
    imu_sample_new.delta_vel_clipping = logical([0 0 0]);
    updated = setIMUData(imu_sample_new);

    
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

