

clear 
foldername = 'D:\桌面\测试数据\八月份数据\0813\';
filename = 'N128_2024-08-13_10-36-20.log';
data = Parse_Data(foldername,filename);
%%
% imu_t = double(data.IMU0.t)/1e6;
% imu_len = length(imu_t);
% imu_dt = 0.002*ones(imu_len,1);
% imu_dt(1:end-1,1) = diff(imu_t);
% AX = data.IMU0.AX;
% AY = data.IMU0.AY;
% AZ = data.IMU0.AZ;
% 
% states.quat_nominal = single([1 0 0 0]);
% states.vel = single([0 0 0]);
% states.pos = single([0 0 0]);
% states.delta_ang_bias = single([0 0 0]);
% states.delta_vel_bias = single([0 0 0]);
% clear all
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


imu.time_us = uint64(0);
imu.delta_ang = single([0 0 0]');
imu.delta_vel = single([1 0 0]');
imu.delta_ang_dt = single(0);
imu.delta_vel_dt = single(0);
imu_sample_delayed = imu;
params.imu_pos_body = single([1 0 0]');
params.vel_Tau = single(0.25);
params.pos_Tau = single(0.25);


len = length(data.IMU0.t);
imu_t = double(data.IMU0.t)/1e6;
imu_dt = 0.002*ones(len,1);
imu_dt(2:end,1) = diff(imu_t);
imu_gyro = [data.IMU0.GY data.IMU0.GX -data.IMU0.GZ];
imu_acc = [data.IMU0.AY data.IMU0.AX -data.IMU0.AZ];
imu_delta_ang = imu_gyro.*imu_dt;
imu_delta_vel = imu_acc.*imu_dt;
correct_update = logical(false);
quat_new = single(zeros(len,4));
vel_new = single(zeros(len,3));
pos_new = single(zeros(len,3));
for i = 1:len
    imu.time_us = data.IMU0.t(i,1);
    imu.delta_ang = imu_delta_ang(i,:)';
    imu.delta_vel = imu_delta_vel(i,:)';
    imu.delta_ang_dt = imu_dt(i,1);
    imu.delta_vel_dt = imu_dt(i,1);
    if correct_update
        predictState();
    end
    
    output_new = calculateOutputStates(imu,imu_sample_delayed,params,correct_update);
    quat_new(i,:) = output_new.quat_nominal;
    vel_new(i,:) = output_new.vel;
    pos_new(i,:) = output_new.pos;
end
clear calculateOutputStates;
%%  传感器数据检查
figure
subplot(311)
plot(imu_t,imu_delta_ang(:,1));
subplot(312)
plot(imu_t,imu_delta_ang(:,2));
subplot(313)
plot(imu_t,imu_delta_ang(:,3));
%% 四元数检查
pcmd_t = double(data.ESKF.StartTime)/1e6;
len2 = length(pcmd_t);
euler = zeros(len2,3);
for i = 1:len2
    euler(i,:) = quattoeuler([data.ESKF.Q0(i,1) data.ESKF.Q1(i,1) data.ESKF.Q2(i,1) data.ESKF.Q3(i,1)]);
end
figure
subplot(311)
plot(pcmd_t,euler(:,1),pcmd_t,data.ESKF.roll);
subplot(312)
plot(pcmd_t,euler(:,2),pcmd_t,data.ESKF.pitch)
subplot(313)
plot(pcmd_t,euler(:,3),pcmd_t,data.ESKF.yaw)
% for i = 1:len
%     euler = quat2eul(quat_new(i,:));
% end
%% 速度位置检查
figure
subplot(211);
plot(imu_t,vel_new(:,1),imu_t,vel_new(:,2),imu_t,vel_new(:,3));
subplot(212)
plot(imu_t,pos_new(:,1),imu_t,pos_new(:,2),imu_t,pos_new(:,3))