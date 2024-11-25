run("init.m");
%%
close all
clear setIMUData calculateOutputStates predictState predictCovariance;

quat_angle_out = zeros(len,4);
ang_out = zeros(len,3);
vel_out = zeros(len,3);

delta_ang_dt_avg = 0.002;
target_dt_s = saturation(params.filter_update_interval_us,1000, 100000) * 1e-6;		%0703:4000 0831:8000
required_samples = max(round(target_dt_s / delta_ang_dt_avg), 1);					%0.004
target_dt_s = required_samples * delta_ang_dt_avg;
min_dt_s = max(delta_ang_dt_avg * (required_samples - 1), delta_ang_dt_avg * 0.5);

ang_out_display = single(zeros(len,3));
vel_out_display = single(zeros(len,3));
quat_angle_out_display = single(zeros(len,4));

delta_angle_display = single(zeros(len,3));
dq_display = single(zeros(len,4));
delta_angle_corr_out_display = zeros(len,3);
quat_delta_delay_display = single(zeros(len,4));
quat_nominal_inverse_display = single(zeros(len,4));
vel_correction_display = zeros(len,3);
pos_correction_display = zeros(len,3);

states_quat_nominal = single(zeros(len,4));
states_vel = single(zeros(len,3));
states_pos = single(zeros(len,3));
states_vel_predict = single(zeros(len,3));
states_pos_predict = single(zeros(len,3));
dq_dt_display = single(zeros(len,4));
states_quat_nominal(1,:) = single([1 0 0 0]);
KF_P = single(zeros(23,23,len));
KF_P_predict = single(zeros(23,23,len));
figure('Name','P_M')
h_m = surf(P_M);
shading interp; % 平滑着色
colormap(jet);
colorbar;
xlabel('X 轴');
ylabel('Y 轴');
zlabel('Z 轴');
title('P_M 动态演示');

figure('Name','P')
h = surf(P);
shading interp; % 平滑着色
colormap(jet);
colorbar;
xlabel('X 轴');
ylabel('Y 轴');
zlabel('Z 轴');
title('P 动态演示');
% gps_index_display = zeros(len,1);


K_Display = zeros(23,23);

K_figure = zeros(23,23);
figure('Name','K')
h_k = surf(K_figure);
shading interp; % 平滑着色
colormap(jet);
colorbar;
xlabel('X 轴');
ylabel('Y 轴');
zlabel('Z 轴');
title('K 动态演示');


for i = 1:len
    imu.time_us = data.IMU1.t(i,1);
    imu.delta_ang = imu_delta_ang(i,:)';
    imu.delta_vel = imu_delta_vel(i,:)';
    imu.delta_ang_dt = imu_dt(i,1);
    imu.delta_vel_dt = imu_dt(i,1);

%     setBaroData(data.BAR0)
%     baro_dt = data.BAR0.t-imu.time_us;
%     baro_index = find(baro_dt<1e3,1,"last");
%     baro.time_us = data.BAR0.t(baro_index,:);
%     baro.hgt = data.BAR0.Hight(baro_index,:);
    [correct_update] = setIMUData(imu,required_samples,target_dt_s,min_dt_s);
    ang_out_display(i,:) = ang_out;
    vel_out_display(i,:) = vel_out;
    quat_angle_out_display(i,:) = quat_angle_out;

    dq_dt = single([1 0 0 0]);
    if correct_update

        predictState(params,CONSTANTS_ONE_G);

        states_vel_predict(i,:) = states.vel';
        states_pos_predict(i,:) = states.pos';

        predictCovariance(params,control_status);
        predictCovariance_Matrix(params);

%        yawEstimator.Update(CONSTANTS_ONE_G);

        Kfusion4 = zeros(23,1);
        Kfusion5 = zeros(23,1);
        Kfusion6 = zeros(23,1);
        
        controlGpsFusion(data.RTK,imu.time_us,params);
        
        K_Display(:,4:6) = [Kfusion4 Kfusion5 Kfusion6];

%         set(h_k,'ZData',K_Display)
%         pause(0.01);
%         baro_fuse(baro);
%         gps_index_display(i,1) = gps_index;
    end
    
    dq_dt_display(i,:) = dq_dt;
    states_quat_nominal(1,:) = states.quat_nominal';
    states_vel(i,:) = states.vel';
    states_pos(i,:) = states.pos';
%     KF_P(:,:,i) = P;
    set(h_m,'ZData',P_M)
    pause(0.01);
    set(h,'ZData',P)
    pause(0.1);
%     correct_update = logical(false);
%     %一直运行更新的变量
%     dq = single([1 0 0 0]);

    % updated 后变量
    quat_delta_delay = single([1 0 0 0]);
    quat_nominal_inverse = single([1 0 0 0]);
    vel_correction = single([0 0 0]');
    pos_correction = single([0 0 0]');
    delta_angle_corr = single([0 0 0]');

    [output_new] = calculateOutputStates(imu,params,correct_update,CONSTANTS_ONE_G);
    
    delta_angle_display(i,:) = delta_angle;
    dq_display(i,:) = dq;
    quat_delta_delay_display(i,:) = quat_delta_delay;
    quat_nominal_inverse_display(i,:) = quat_nominal_inverse; 
    delta_angle_corr_out_display(i,:) = delta_angle_corr;
    vel_correction_display(i,:) = vel_correction;
    pos_correction_display(i,:) = pos_correction;

    quat_new(i,:) = output_new.quat_nominal;
    vel_new(i,:) = output_new.vel;
    pos_new(i,:) = output_new.pos;
 
end
%%
t =  double(data.IMU1.t)/1e6;
%%  setIMUData check
figure('Name','imu data')
subplot(311)
plot(t,quat_angle_out(:,1),t,quat_angle_out(:,2),t,quat_angle_out(:,3),t,quat_angle_out(:,4))
subplot(312)
plot(t,ang_out(:,1),'*',t,ang_out(:,2),t,ang_out(:,3));
subplot(313)
plot(t,vel_out(:,1),'*',t,vel_out(:,2),t,vel_out(:,3));

%% predictState check
figure('Name','dq_dt')
subplot(211)
plot(t,dq_dt_display(:,1),'*')
subplot(212)
plot(t,dq_dt_display(:,2),'*',t,dq_dt_display(:,3),t,dq_dt_display(:,4))
%%
figure('Name','state vel pos')
plot(t,states_vel(:,1),t,states_vel(:,2),t,states_vel(:,3));
figure
plot(t,states_pos(:,1),t,states_pos(:,2),t,states_pos(:,3))
%% calculateOutputStates check
figure('Name','delta_angle_display')
plot(t,delta_angle_display(:,1),t,delta_angle_display(:,2),t,delta_angle_display(:,3));
%%
figure('Name','dq')
subplot(211)
plot(t,dq_display(:,1))
subplot(212)
plot(t,dq_display(:,2),'*',t,dq_display(:,3),t,dq_display(:,4))
%%
figure('Name','quat')
subplot(211)
plot(t,quat_new(:,1))
subplot(212)
plot(t,quat_new(:,2),'*',t,quat_new(:,3),t,quat_new(:,4))
%%
figure
subplot(211)
plot(t,quat_nominal_inverse_display(:,1))
subplot(212)
plot(t,quat_nominal_inverse_display(:,2),t,quat_nominal_inverse_display(:,3),t,quat_nominal_inverse_display(:,4))

%%
figure
subplot(211)
plot(t,quat_delta_delay_display(:,1))
subplot(212)
plot(t,quat_delta_delay_display(:,2),'*',t,quat_delta_delay_display(:,3),t,quat_delta_delay_display(:,4))


%%

figure('Name','delta_angle_corr_out')
plot(t,delta_angle_corr_out_display(:,1),t,delta_angle_corr_out_display(:,2),t,delta_angle_corr_out_display(:,3));
figure('Name','vel_correction')
plot(t,vel_correction_display(:,1),t,vel_correction_display(:,2),t,vel_correction_display(:,3));
figure('Name','pos_correction')
plot(t,pos_correction_display(:,1),t,pos_correction_display(:,2),t,pos_correction_display(:,3));
%% 四元数检查



%% 欧拉角检查
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
plot(pcmd_t,euler(:,2),eskf_t,data.ESKF.pitch);
subplot(313)
plot(pcmd_t,euler(:,3),eskf_t,data.ESKF.yaw);

%% 速度位置检查
figure
subplot(211);
plot(imu_t,vel_new(:,1),imu_t,vel_new(:,2),imu_t,vel_new(:,3));
subplot(212)
plot(imu_t,pos_new(:,1),imu_t,pos_new(:,2),imu_t,pos_new(:,3))