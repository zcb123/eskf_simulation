

clear 
foldername = 'C:\Users\89565\Desktop\自研飞控\TestData\十一月数据\1119\静态试验\';
filename = 'N41_2024-11-19_11-19-29.log';
data = Parse_Data(foldername,filename);
%%

run("init.m");

for i = 1:len
    imu.time_us = data.IMU1.t(i,1);
    imu.delta_ang = imu_delta_ang(i,:)';
    imu.delta_vel = imu_delta_vel(i,:)';
    imu.delta_ang_dt = imu_dt(i,1);
    imu.delta_vel_dt = imu_dt(i,1);
    imu_sample_delayed = imu;
    if correct_update
        predictState(imu_sample_delayed,params,CONSTANTS_ONE_G);
    end
    
    output_new = calculateOutputStates(imu,imu_sample_delayed,params,correct_update,CONSTANTS_ONE_G);
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
figure
plot(imu_dt)
%% 四元数检查
pcmd_t = double(data.PCMD.t)/1e6;

len2 = length(pcmd_t);
euler = zeros(len2,3);
for i = 1:len2
    euler(i,:) = quattoeuler([quat_new(i,1) quat_new(i,2) quat_new(i,3) quat_new(i,4)]);
end

figure
subplot(221)
plot(quat_new(:,1))
subplot(222)
plot(quat_new(:,2))
subplot(223)
plot(quat_new(:,3))
subplot(224)
plot(quat_new(:,4))
%%
eskf_t = double(data.ESKF.t)/1e6;
figure
subplot(311)
plot(pcmd_t,euler(:,1),eskf_t,data.ESKF.roll);
subplot(312)
plot(pcmd_t,euler(:,2),eskf_t,data.ESKF.pitch)
subplot(313)
plot(pcmd_t,euler(:,3),eskf_t,data.ESKF.yaw)
% for i = 1:len
%     euler = quat2eul(quat_new(i,:));
% end
%% 速度位置检查
figure
subplot(211);
plot(imu_t,vel_new(:,1),imu_t,vel_new(:,2),imu_t,vel_new(:,3));
subplot(212)
plot(imu_t,pos_new(:,1),imu_t,pos_new(:,2),imu_t,pos_new(:,3))