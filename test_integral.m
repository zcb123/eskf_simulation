
load("data/N41_2024-11-19_11-19-29.mat");

len = length(data.IMU1.t);
imu_t = double(data.IMU1.t)/1e6;
imu_dt = 0.002*ones(len,1);
imu_dt(2:end,1) = diff(imu_t);
imu_gyro = [data.IMU1.GY data.IMU1.GX -data.IMU1.GZ];
imu_acc = [data.IMU1.AY data.IMU1.AX -data.IMU1.AZ];
imu_acc_integral = zeros(len,3);
imu_acc_integral_dt = zeros(len,1);
imu_acc_integral_output = zeros(len,3);
imu_gyro_integral = zeros(len,3);
imu_gyro_integral_dt = zeros(len,1);
imu_gyro_integral_output = zeros(len,3);



imu_acc_filted = zeros(len,3);
index = 1;
clear acc_integral gyro_coningIntegral
for i = 1:len
    
    [imu_acc_integral(i,:),imu_acc_integral_dt(i,1),acc_flag] = acc_integral(imu_acc(i,:),imu_dt(i,1));
    acc_dt_inv = 1e6/imu_acc_integral_dt(i,1);
    imu_acc_integral_output(i,:) = imu_acc_integral(i,:)*acc_dt_inv;
%     将未更新的部分去掉

    [imu_gyro_integral(i,:),imu_gyro_integral_dt(i,1),gyro_flag] = gyro_coningIntegral(imu_gyro(i,:),imu_dt(i,1));
    gyro_dt_inv = 1e6/imu_gyro_integral_dt(i,1);
    imu_gyro_integral_output(i,:) = imu_gyro_integral(i,:)*gyro_dt_inv;
    
    if acc_flag && gyro_flag
        [acc_lpf,imu_acc_filted(index,:)] = acc_lpf.apply(imu_acc_integral_output(i,:));
        index = index + 1;
    end

end
imu_acc_integral_output_x = imu_acc_integral_output(:,1);
imu_acc_update_x = imu_acc_integral_output_x(imu_acc_integral_output_x~=0);
% imu_acc_update_dt = imu_acc_integral_dt(imu_acc_integral_dt~=0);
%%
% [fAz,pAz]=eust_fft(data.IMU1.t,data.IMU1.AZ);
% [fAzInt,pAzInt]=eust_fft(data.IMU1.t,imu_acc_integral_output);
% figure
% plot(fAz,pAz,fAzInt,pAzInt)

%%
figure
plot(imu_t,imu_acc_filted(:,1));
% figure
% plot(imu_acc_update_dt,imu_acc_update()
%%
% figure
% subplot(311)
% plot(imu_t,imu_acc(:,1),'*-')
% hold on 
% plot(imu_t,imu_acc_integral_output(:,1),'*-');
% subplot(312)
% plot(imu_t,imu_acc(:,2),'*-')
% hold on 
% plot(imu_t,imu_acc_integral_output(:,2),'*-');
% subplot(313)
% plot(imu_t,imu_acc(:,3),'*-')
% hold on 
% plot(imu_t,imu_acc_integral_output(:,3),'*-');

%%
figure
plot(imu_t,imu_gyro(:,1),'*-',imu_t,imu_gyro_integral_output(:,1),'*-');
% figure
% subplot(311)
% plot(imu_t,imu_gyro(:,1),'*-',imu_t,imu_gyro_integral_output(:,1),'*-');
% grid on
% subplot(312)
% plot(imu_t,imu_gyro(:,2),'*-',imu_t,imu_gyro_integral_output(:,2),'*-');
% grid on
% subplot(313)
% plot(imu_t,imu_gyro(:,3),'*-',imu_t,imu_gyro_integral_output(:,3),'*-');
% grid on


