
load("data/N41_2024-11-19_11-19-29.mat");

len = length(data.IMU1.t);
imu_t = double(data.IMU1.t)/1e6;
imu_dt = 0.002*ones(len,1);
imu_dt(2:end,1) = diff(imu_t);
imu_gyro = [data.IMU1.GY data.IMU1.GX -data.IMU1.GZ];
imu_acc = [data.IMU1.AY data.IMU1.AX -data.IMU1.AZ];
imu_acc_integral = zeros(len,3);
imu_acc_integral_dt = zeros(len,1);

clear acc_integral
for i = 1:len
    [imu_acc_integral(i,:),imu_acc_integral_dt(i,1),flag] = acc_integral(imu_acc(i,:),imu_dt(i,1));
end

%%
figure
plot(imu_acc(:,1))
hold on 
plot(imu_acc_integral(:,1))




