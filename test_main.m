

run("init.m");
%%


clear setIMUData calculateOutputStates predictState predictCovariance;
% dqfigure = figure('Name','dq');
dq = zeros(len,4);
delta_angle_corr_out = zeros(len,3);
vel_correction = zeros(len,3);
pos_correction = zeros(len,3);

delta_ang_dt_avg = 0.002;
target_dt_s = saturation(params.filter_update_interval_us,1000, 100000) * 1e-6;		%0703:4000 0831:8000
required_samples = max(round(target_dt_s / delta_ang_dt_avg), 1);					%0.004
target_dt_s = required_samples * delta_ang_dt_avg;
% minimum delta angle dt (in addition to number of samples)
min_dt_s = max(delta_ang_dt_avg * (required_samples - 1), delta_ang_dt_avg * 0.5);

for i = 1:len
    imu.time_us = data.IMU1.t(i,1);
    imu.delta_ang = imu_delta_ang(i,:)';
    imu.delta_vel = imu_delta_vel(i,:)';
    imu.delta_ang_dt = imu_dt(i,1);
    imu.delta_vel_dt = imu_dt(i,1);
    correct_update = setIMUData(imu,required_samples,target_dt_s,min_dt_s);
%     if correct_update
%         predictState(params,CONSTANTS_ONE_G);
%         predictCovariance(params,control_status);
%     end
   

    [output_new,dq(i,:),delta_angle_corr_out(i,:),vel_correction(i,:),pos_correction(i,:)] = calculateOutputStates(imu,params,correct_update,CONSTANTS_ONE_G);
    

    quat_new(i,:) = output_new.quat_nominal;
    vel_new(i,:) = output_new.vel;
    pos_new(i,:) = output_new.pos;
    

end
%%
t =  double(data.IMU1.t)/1e6;
figure('Name','dq')
plot(t,dq(:,1),t,dq(:,2),t,dq(:,3),t,dq(:,4));

figure('Name','quat')
plot(t,quat_new(:,1),t,quat_new(:,2),t,quat_new(:,3),t,quat_new(:,4))

figure('Name','delta_angle_corr_out')
plot(t,delta_angle_corr_out(:,1),t,delta_angle_corr_out(:,2),t,delta_angle_corr_out(:,3));
figure('Name','vel_correction')
plot(t,vel_correction(:,1),t,vel_correction(:,2),t,vel_correction(:,3));
figure('Name','pos_correction')
plot(t,pos_correction(:,1),t,pos_correction(:,2),t,pos_correction(:,3));
%% 四元数检查
pcmd_t = double(data.PCMD.t)/1e6;

len2 = length(pcmd_t);
euler = zeros(len2,3);
for i = 1:len2
    euler(i,:) = quattoeuler([quat_new(i,1) quat_new(i,2) quat_new(i,3) quat_new(i,4)]);
end

figure
plot(pcmd_t,euler(:,1),pcmd_t,euler(:,2),pcmd_t,euler(:,3));

%%
eskf_t = double(data.ESKF.t)/1e6;
figure
subplot(311)
plot(pcmd_t,euler(:,1),eskf_t,data.ESKF.roll);
subplot(312)
plot(pcmd_t,euler(:,2),eskf_t,data.ESKF.pitch)
subplot(313)
plot(pcmd_t,euler(:,3),eskf_t,data.ESKF.yaw)

%% 速度位置检查
figure
subplot(211);
plot(imu_t,vel_new(:,1),imu_t,vel_new(:,2),imu_t,vel_new(:,3));
subplot(212)
plot(imu_t,pos_new(:,1),imu_t,pos_new(:,2),imu_t,pos_new(:,3))