clear 
close all

load("data/N162_2024-11-13_21-00-07.mat");

run("global_init.m");

len = length(data.IMU0.t);
imu_t = double(data.IMU0.t)/1e6;
imu_dt = 0.002*ones(len,1);
imu_dt(2:end,1) = diff(imu_t);
% imu_gyro = [data.IMU0.GY data.IMU0.GX -data.IMU0.GZ];
% imu_acc = [data.IMU0.AY data.IMU0.AX -data.IMU0.AZ];
imu_gyro = [data.IMU0.GX data.IMU0.GY data.IMU0.GZ];
imu_acc = [data.IMU0.AX data.IMU0.AY data.IMU0.AZ];

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

% figure('Name','vehivle acc')
% subplot(311)
% plot(imu_t,imu_acc(:,1),vehicle_t,vehicle_acc(:,1))
% grid on 
% subplot(312)
% plot(imu_t,imu_acc(:,2),vehicle_t,vehicle_acc(:,2))
% grid on 
% subplot(313)
% plot(imu_t,imu_acc(:,3),vehicle_t,vehicle_acc(:,3))
% grid on 
% figure('Name','vehivle gyro')
% subplot(311)
% plot(imu_t,imu_gyro(:,1),vehicle_t,vehicle_gyro(:,1))
% grid on 
% subplot(312)
% plot(imu_t,imu_gyro(:,2),vehicle_t,vehicle_gyro(:,2))
% grid on 
% subplot(313)
% plot(imu_t,imu_gyro(:,3),vehicle_t,vehicle_gyro(:,3))
% grid on 

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
    [acc_lpf_2o,acc_filted(i,:)] = acc_lpf_2o.apply([vehicle_acc(i,2) vehicle_acc(i,1) -vehicle_acc(i,3)]);
    [gyro_lpf_2o,gyro_filted(i,:)] = gyro_lpf_2o.apply([vehicle_gyro(i,2) vehicle_gyro(i,1) -vehicle_gyro(i,3)]);

end
%%
figure('Name','acc_filted and frame transform')
subplot(311)
plot(vehicle_t,vehicle_acc(:,2),vehicle_t,acc_filted(:,1))
grid on
legend('aX','aXF');
subplot(312)
plot(vehicle_t,vehicle_acc(:,1),vehicle_t,acc_filted(:,2))
grid on
legend('aY','aYF');
subplot(313)
plot(vehicle_t,-vehicle_acc(:,3),vehicle_t,acc_filted(:,3))
grid on
legend('aZ','aZF');
% 
%%
figure('Name','gyro_filted and frame transform')
subplot(311)
plot(vehicle_t,vehicle_gyro(:,2),vehicle_t,gyro_filted(:,1))
grid on
legend('gX','gXF');
subplot(312)
plot(vehicle_t,vehicle_gyro(:,1),vehicle_t,gyro_filted(:,2))
grid on
legend('gY','gYF');
subplot(313)
plot(vehicle_t,-vehicle_gyro(:,3),vehicle_t,gyro_filted(:,3))
grid on
legend('gZ','gZF');