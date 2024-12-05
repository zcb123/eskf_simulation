clear 
foldername = 'C:\Users\89565\Desktop\自研飞控\TestData\十月份数据\1030\';
filename = 'N447_2024-10-30_17-45-26.log';
data = Parse_Data(foldername,filename);
%%

imu_t = double(data.IMU0.t)/1e6;
imu_t = imu_t - imu_t(1,1);
% ImuData.timestamp = timeseries(data.IMU0.t,imu_t);
ImuData.acc_x_mPs2 = timeseries(data.IMU0.AX,imu_t);
ImuData.acc_y_mPs2 = timeseries(data.IMU0.AY,imu_t);
ImuData.acc_z_mPs2 = timeseries(data.IMU0.AZ,imu_t);
% % ImuData.gyro_dt_us = timeseries()
ImuData.gyro_x_radPs = timeseries(data.IMU0.GX,imu_t);
ImuData.gyro_y_radPs = timeseries(data.IMU0.GY,imu_t);
ImuData.gyro_z_radPs = timeseries(data.IMU0.GZ,imu_t);

elems(1) = Simulink.BusElement;
elems(1).Name = 'acc_x_mPs2';
elems(1).DataType = "single";
elems(2) = Simulink.BusElement;
elems(2).Name = 'acc_y_mPs2';
elems(2).DataType = "single";
elems(3) = Simulink.BusElement;
elems(3).Name = 'acc_z_mPs2';
elems(3).DataType = "single";
elems(4) = Simulink.BusElement;
elems(4).Name = 'gyro_x_radPs';
elems(4).DataType = "single";
elems(5) = Simulink.BusElement;
elems(5).Name = 'gyro_y_radPs';
elems(5).DataType = "single";
elems(6) = Simulink.BusElement;
elems(6).Name = 'gyro_z_radPs';
elems(6).DataType = "single";
ImuDataBus = Simulink.Bus;
ImuDataBus.Elements = elems;

baro_t = double(data.BAR0.t)/1e6;
baro_t = baro_t - baro_t(1,1);
BaroData.height = timeseries(data.BAR0.Hight,baro_t);
elems(1) = Simulink.BusElement;
elems(1).Name = 'height';
elems(1).DataType = "single";
BaroDataBus = Simulink.Bus;
BaroDataBus.Elements = elems;

pcmd_t = double(data.PCMD.t)/1e6;
pcmd_t = pcmd_t - pcmd_t(1,1);
PcmdData.throttle_cmd = timeseries(data.PCMD.thr_cmd,pcmd_t);

dt = mean(diff(imu_t));
sim_time = imu_t(end,1) - imu_t(1,1);