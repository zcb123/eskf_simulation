run("init_setParams.m");


clear setIMUData calculateOutputStates predictState predictCovariance controlGpsFusion;
global states imu_sample_delayed dt_imu_avg  dt_ekf_avg P P_M;


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
% nonNaN_index = ~isnan(imu_acc_update);
% nonNaN_num = sum(nonNaN_index(:,1));
% vehicle_acc = reshape(imu_acc_update(nonNaN_index),[nonNaN_num,3]);
vehicle_acc = getNonNaN(imu_acc_update,3);
vehicle_gyro = getNonNaN(imu_gyro_update,3);
vehicle_t = getNonNaN(imu_t_update,1);
%%
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

%% 二阶低通滤波
len_t = length(vehicle_t);
acc_filted = zeros(len_t,3);
gyro_filted = zeros(len_t,3);

for i=1:len_t
    [acc_lpf,acc_filted(i,:)] = acc_lpf.apply(vehicle_acc(i,:));
    [gyro_lpf,gyro_filted(i,:)] = gyro_lpf.apply(vehicle_gyro(i,:));

end

%%
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
%% IMU下采样
clear setIMUData;

reset_setIMUData(0.0025);


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

dt_imu_avg_record = zeros(len_t,1);
index = 1;
for i = 1:len_t

    imu_sample_new.time_us = uint64(vehicle_t(i,1)*1e6);   %转化成微秒
    imu_sample_new.delta_ang = gyro_filted(i,:)'*vehicle_dt(i,1);
    imu_sample_new.delta_vel = acc_filted(i,:)'*vehicle_dt(i,1);
    imu_sample_new.delta_ang_dt = vehicle_dt(i,1);
    imu_sample_new.delta_vel_dt = vehicle_dt(i,1);
    imu_sample_new.delta_vel_clipping = logical([0 0 0]);
    
    imu_sample_delta_ang(i,:) = imu_sample_new.delta_ang';
    imu_sample_delta_vel(i,:) = imu_sample_new.delta_vel';

    updated = setIMUData(imu_sample_new);
    
    dt_imu_avg_record(i,1) = dt_imu_avg;
    ang_out_display(i,:) = ang_out';
    vel_out_display(i,:) = vel_out';
    quat_angle_out_display(i,:) = quat_angle_out;

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
figure
plot(vehicle_t,dt_imu_avg_record(:,1));
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
%%
% figure
% subplot(311)
% plot(vehicle_t,imu_sample_delta_ang(:,1),'*-',imu_delta_t,imu_delta_ang(:,1),'*-')
% grid on
% legend('deltaAng','deltaAngDownSample')
% subplot(312)
% plot(vehicle_t,imu_sample_delta_ang(:,2),'*-',imu_delta_t,imu_delta_ang(:,2),'*-')
% grid on
% legend('deltaAng','deltaAngDownSample')
% subplot(313)
% plot(vehicle_t,imu_sample_delta_ang(:,3),'*-',imu_delta_t,imu_delta_ang(:,3),'*-')
% grid on
% legend('deltaAng','deltaAngDownSample')
% 
% figure
% subplot(311)
% plot(vehicle_t,imu_sample_delta_vel(:,1),'*-',imu_delta_t,imu_delta_vel(:,1),'*-')
% grid on
% legend('deltaVel','deltaVelDownSample')
% subplot(312)
% plot(vehicle_t,imu_sample_delta_vel(:,2),'*-',imu_delta_t,imu_delta_vel(:,2),'*-')
% grid on
% legend('deltaVel','deltaVelDownSample')
% subplot(313)
% plot(vehicle_t,imu_sample_delta_vel(:,3),'*-',imu_delta_t,imu_delta_vel(:,3),'*-')
% grid on
% legend('deltaVel','deltaVelDownSample')
% 
% figure
% subplot(211)
% plot(imu_ang_dt)
% legend('imuAngDt')
% subplot(212)
% plot(imu_vel_dt)
% legend('imuVelDt')
%% 在进入预测之前的数据变化

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
clear delta_ang_dt_avg target_dt_s required_samples target_dt_s min_dt_s...
    imu_sample_delta_ang  imu_sample_delta_vel quat_angle_out ...
    ang_out_display vel_out_display quat_angle_out_display ...
    imu_sample_delayed_t imu_sample_delayed_ang imu_sample_delayed_vel ...
    imu_sample_ang_dt imu_sample_vel_dt;


%% 预测
len_delta_t = length(imu_delta_t);

states_quat_nominal = zeros(len_delta_t,4);
states_vel_predict = zeros(len_delta_t,3);
states_pos_predict = zeros(len_delta_t,3);

delta_ang_display = zeros(len_delta_t,3);

init_states();

euler = zeros(len_delta_t,3);

for i = 1:len_delta_t

    imu_sample_updated.delta_ang = imu_delta_ang(i,:)';
    imu_sample_updated.delta_vel = imu_delta_vel(i,:)';
    imu_sample_updated.delta_vel_dt = imu_vel_dt(i,1);
    imu_sample_updated.delta_ang_dt = imu_ang_dt(i,1);

    predictState(imu_sample_updated);

    euler(i,:) = quattoeuler([states.quat_nominal(1) states.quat_nominal(2) states.quat_nominal(3) states.quat_nominal(4)]);

    delta_ang_display(i,:) = delta_ang';
    states_quat_nominal(i,:) = states.quat_nominal';
    states_vel_predict(i,:) = states.vel';
    states_pos_predict(i,:) = states.pos';
end


%%
% figure
% plot(imu_delta_t,imu_delta_ang(:,1),imu_delta_t,delta_ang_display(:,1));
% 
% figure
% subplot(211)
% plot(imu_delta_t,states_quat_nominal(:,1))
% subplot(212)
% plot(imu_delta_t,states_quat_nominal(:,2),imu_delta_t,states_quat_nominal(:,3),imu_delta_t,states_quat_nominal(:,4))
% 
% figure
% plot(imu_delta_t,euler(:,1),imu_delta_t,euler(:,2),imu_delta_t,euler(:,3))
% figure('Name','vel predict')
% subplot(311)
% plot(imu_delta_t,states_vel_predict(:,1))
% subplot(312)
% plot(imu_delta_t,states_vel_predict(:,2))
% subplot(313)
% plot(imu_delta_t,states_vel_predict(:,3))
% 
% figure('Name','pos predict')
% subplot(311)
% plot(imu_delta_t,states_pos_predict(:,1))
% subplot(312)
% plot(imu_delta_t,states_pos_predict(:,2))
% subplot(313)
% plot(imu_delta_t,states_pos_predict(:,3))
%%
clear imu_sample_updated dq_dt delta_ang delta_ang_display states_quat_nominal states_vel_predict states_pos_predict euler;
%% 预测的协方差
len_delta_t = length(imu_delta_t);

init_states();

run("P_init.m");

global delta_angle_var_accum delta_vel_var_accum

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



delta_ang_var_accum_display = zeros(len_delta_t,3);
delta_vel_var_accum_display = zeros(len_delta_t,3);

for i = 1:len_delta_t

    imu_sample_updated.delta_ang = imu_delta_ang(i,:)';
    imu_sample_updated.delta_vel = imu_delta_vel(i,:)';
    imu_sample_updated.delta_vel_dt = imu_vel_dt(i,1);
    imu_sample_updated.delta_ang_dt = imu_ang_dt(i,1);
    imu_sample_updated.delta_ang_clipping = logical([0 0 0]);
    imu_sample_updated.delta_vel_clipping = logical([0 0 0]);
    
    predictState(imu_sample_updated);
    predictCovariance(imu_sample_updated);
    
    predictCovariance_Matrix(imu_sample_updated);

    delta_ang_var_accum_display(i,:) = delta_angle_var_accum;
    delta_vel_var_accum_display(i,:) = delta_vel_var_accum;

    delta_P = P_M - P;

%     set(h,'ZData',P)
%     pause(0.01);
%     set(h_m,'ZData',P_M)
%     pause(0.01);

end
%%
figure('Name','delta_ang_var_accum_display')
plot(imu_delta_t,delta_ang_var_accum_display(:,1),imu_delta_t,delta_ang_var_accum_display(:,2),imu_delta_t,delta_ang_var_accum_display(:,3));
%%
figure('Name','delta_vel_var_accum_display')
plot(imu_delta_t,delta_vel_var_accum_display(:,1),imu_delta_t,delta_vel_var_accum_display(:,2),imu_delta_t,delta_vel_var_accum_display(:,3))
%% EKFGSF_YAW 检查
clear predictState predictCovariance predictCovariance_Matrix ...
    fuseVelocityWithLevelArm fuseVelocityWithLevelArm_Matrix;   

clear yawEstimator;
global yawEstimator;
yawEstimator = EKFGSF_YAW();

close all

len_delta_t = length(imu_delta_t);
init_states();
run("P_init.m");

gps_index_last = 0;
ahrs_accel_norm_display = zeros(len_delta_t,1);
ahrs_accel_fusion_gain_display = zeros(len_delta_t,1);
gsf_yaw_display = zeros(len_delta_t,1);

% P_GSF = zeros(3,3);
% figure('Name','P_GSF')
% p_m = surf(P_GSF);
% shading interp; % 平滑着色
% colormap(jet);
% colorbar;
% xlabel('X 轴');
% ylabel('Y 轴');
% zlabel('Z 轴');
% title('P_GSF 动态演示');

% R_GSF = zeros(3,3);
% figure('Name','R_GSF')
% rot = surf(R_GSF);
% shading interp; % 平滑着色
% colormap(jet);
% colorbar;
% xlabel('X 轴');
% ylabel('Y 轴');
% zlabel('Z 轴');
% title('R_GSF 动态演示');

K_display = zeros(3,2);
% R_GSF = zeros(3,3);
figure('Name','K_GSF')
k = surf(K_display);
shading interp; % 平滑着色
colormap(jet);
colorbar;
xlabel('X 轴');
ylabel('Y 轴');
zlabel('Z 轴');
title('K_GSF 动态演示');

delta_angle_corrected_display = zeros(len_delta_t,3);
delta_angle_corrected = [0 0 0]';
test_ratio = 0;
test_ratio_display = zeros(len_delta_t,1);
model_weights_display = zeros(len_delta_t,5);
X1_display = zeros(len_delta_t,1);
X2_display = zeros(len_delta_t,1);
X3_display = zeros(len_delta_t,5);
gravity_direction_bf_display = zeros(len_delta_t,3);
accel_display = zeros(len_delta_t,3);
gravity_direction_bf = [0 0 0]';
accel = [0 0 0]';

spinRate_display = zeros(len_delta_t,1);
tilt_correction_display = zeros(len_delta_t,3);
gyro_bias_display = zeros(len_delta_t,3);
spinRate = 0;
tilt_correction = [0 0 0]';
gyro_bias = [0 0 0]';
R_display=zeros(3,3,len_delta_t);
index_display = zeros(len_delta_t,3);

normDist_display = zeros(len_delta_t,1);
normDist = 0;
yaw_variance_display = zeros(len_delta_t,1);
innov_display = zeros(len_delta_t,2);
K = zeros(3,2);
yawdelta_display = zeros(len_delta_t,1);
yawDelta = 0;
for i = 1:len_delta_t

    imu_sample_updated.delta_ang = imu_delta_ang(i,:)';
    imu_sample_updated.delta_vel = imu_delta_vel(i,:)';
    imu_sample_updated.delta_vel_dt = imu_vel_dt(i,1);
    imu_sample_updated.delta_ang_dt = imu_ang_dt(i,1);
    imu_sample_updated.delta_ang_clipping = logical([0 0 0]);
    imu_sample_updated.delta_vel_clipping = logical([0 0 0]);
    
    predictCovariance(imu_sample_updated);  
    predictCovariance_Matrix(imu_sample_updated);
    predictState(imu_sample_updated);

    gps_data_ready = false;
    gps_dt = data.RTK.t - imu_delta_t(i,1)*1e6;
    gps_index = find(gps_dt<1e3,1,'last');
    if gps_index_last~=gps_index    %目前都默认gps数据是能用的
        % 找时间戳滞后但又不至于滞后太多的
        if gps_dt(gps_index,1)<imu_delta_t(i,1)*1e6 && imu_delta_t(i,1)*1e6 < gps_dt(gps_index,1)+1e5
            gps_data_ready = true;
        end
        gps_index_last = gps_index;
    end
%     gps_data_ready = false;

    runYawEKFGSF(imu_sample_updated,data.RTK,gps_data_ready,gps_index);
    
    set(k,'ZData',K);
    pause(0.01);
    yawdelta_display(i,:) = yawDelta;
    innov_display(i,:) = [yawEstimator.ekf_gsf(1,1).innov(1) yawEstimator.ekf_gsf(1,1).innov(2)];
%     set(p_m,'ZData',yawEstimator.ekf_gsf(1,1).P);
%     pause(0.01);

%     set(rot,'ZData',yawEstimator.ahrs_ekf_gsf(1,1).R);
%     pause(0.01);

    ahrs_accel_norm_display(i,1) = yawEstimator.ahrs_accel_norm;
    ahrs_accel_fusion_gain_display(i,1) = yawEstimator.ahrs_accel_fusion_gain;
    gsf_yaw_display(i,1) = yawEstimator.gsf_yaw;
    delta_angle_corrected_display(i,:) = delta_angle_corrected;
    test_ratio_display(i,1) = test_ratio;
%     P_GSF = yawEstimator.ekf_gsf(1,1).P;
%     set(p_m,'ZData',P_GSF)
%     pause(0.01);
    gravity_direction_bf_display(i,:) = gravity_direction_bf';
    accel_display(i,:) = accel';
    model_weights_display(i,:) = yawEstimator.model_weights';

    spinRate_display(i,1) = spinRate;
    tilt_correction_display(i,:) = tilt_correction;
    gyro_bias_display(i,:) = gyro_bias;
    normDist_display(i,1) = normDist;
    X1_display(i,1) = yawEstimator.ekf_gsf(1,1).X(1);
    X2_display(i,1) = yawEstimator.ekf_gsf(1,1).X(2);
    X3_display(i,:) = [yawEstimator.ekf_gsf(1,1).X(3) yawEstimator.ekf_gsf(2,1).X(3) yawEstimator.ekf_gsf(3,1).X(3)...
        yawEstimator.ekf_gsf(4,1).X(3) yawEstimator.ekf_gsf(5,1).X(3)];
    R_display(:,:,i) = yawEstimator.ahrs_ekf_gsf(1,1).R;
    yaw_variance_display(i,:) = yawEstimator.gsf_yaw_variance;
%     index = find(eye(3)-R_display(:,:,i)*R_display(:,:,i)'~=0)

%     index_display(i,:) = index;
end
%%
figure
plot(yaw_variance_display(:,1));
%%
figure
plot(yawdelta_display);
%%
figure('Name','ahrs_accel_norm_display')
plot(imu_delta_t,ahrs_accel_norm_display);
%%
figure('Name','ahrs_accel_fusion_gain_display')
plot(imu_delta_t,ahrs_accel_fusion_gain_display);
%%
% figure
% plot(imu_delta_t,imu_delta_ang(:,1),imu_delta_t,imu_delta_ang(:,2),imu_delta_t,imu_delta_ang(:,3));
% figure
% plot(imu_delta_t,imu_ang_dt);
% figure
% plot(imu_delta_t,imu_delta_ang(:,1)./imu_ang_dt,imu_delta_t,imu_delta_ang(:,2)./imu_ang_dt,imu_delta_t,imu_delta_ang(:,3)./imu_ang_dt);
%%
figure
plot(imu_delta_t,spinRate_display);
figure('Name','gyro_bias_display')
plot(imu_delta_t,gyro_bias_display(:,1),imu_delta_t,gyro_bias_display(:,2),imu_delta_t,gyro_bias_display(:,3));
figure('Name','tilt_correction_display')
plot(imu_delta_t,tilt_correction_display(:,1),imu_delta_t,tilt_correction_display(:,2),imu_delta_t,tilt_correction_display(:,3))
figure('Name','delta_angle_corrected_display')
plot(imu_delta_t,delta_angle_corrected_display(:,1),imu_delta_t,delta_angle_corrected_display(:,2),imu_delta_t,delta_angle_corrected_display(:,3))
%%
figure('Name','gravity_direction_bf_display')
plot(imu_delta_t,gravity_direction_bf_display(:,1),imu_delta_t,gravity_direction_bf_display(:,2),imu_delta_t,gravity_direction_bf_display(:,3));

figure('Name','accel')
plot(imu_delta_t,accel_display(:,1),imu_delta_t,accel_display(:,2),imu_delta_t,accel_display(:,3));
%%
figure
plot(imu_delta_t,normDist_display)

%%
figure('Name','gsf_yaw_display')
plot(imu_delta_t,gsf_yaw_display*57.3);
hold on
plot(imu_delta_t,X3_display*57.3);
%%
figure
subplot(311)
plot(imu_delta_t,X1_display);
grid on;legend('X1');
subplot(312)
plot(imu_delta_t,X2_display);
grid on;legend('X2');
subplot(313)
plot(imu_delta_t,X3_display);
grid on;legend('X3');
%%

%%
figure
plot(test_ratio_display)
%%

%%

%%
figure
plot(imu_delta_t,delta_angle_corrected_display(:,1),imu_delta_t,imu_delta_ang(:,1))
%% 误差状态更新


% clear predictState predictCovariance predictCovariance_Matrix ...
%     fuseVelocityWithLevelArm fuseVelocityWithLevelArm_Matrix;   
% 
% len_delta_t = length(imu_delta_t);
% 
% init_states();
% 
% run("P_init.m");
% 
% clear yawEstimator;
% global yawEstimator;
% yawEstimator = EKFGSF_YAW();
% % figure('Name','K_M')
% % h_m = surf(P_M);
% % shading interp; % 平滑着色
% % colormap(jet);
% % colorbar;
% % xlabel('X 轴');
% % ylabel('Y 轴');
% % zlabel('Z 轴');
% % title('P_M 动态演示');
% % 
% % figure('Name','K')
% % h = surf(P);
% % shading interp; % 平滑着色
% % colormap(jet);
% % colorbar;
% % xlabel('X 轴');
% % ylabel('Y 轴');
% % zlabel('Z 轴');
% % title('P 动态演示');
% K4_Display = zeros(23,len_delta_t);
% 
% 
% % delta_ang_var_accum_display = zeros(len_delta_t,3);
% % delta_vel_var_accum_display = zeros(len_delta_t,3);
% len_gps = length(data.RTK.t);
% % gps_index_display = zeros(len_gps,1);
% states_quat_nominal_display=zeros(4,len_delta_t);
% states_vel_display=zeros(3,len_delta_t);
% states_pos_display=zeros(3,len_delta_t);
% 
% 
% 
% gps_index_last = 0;
% 
% 
% for i = 1:len_delta_t
% 
%     imu_sample_updated.delta_ang = imu_delta_ang(i,:)';
%     imu_sample_updated.delta_vel = imu_delta_vel(i,:)';
%     imu_sample_updated.delta_vel_dt = imu_vel_dt(i,1);
%     imu_sample_updated.delta_ang_dt = imu_ang_dt(i,1);
%     imu_sample_updated.delta_ang_clipping = logical([0 0 0]);
%     imu_sample_updated.delta_vel_clipping = logical([0 0 0]);
%     
%     predictCovariance(imu_sample_updated);  
%     predictCovariance_Matrix(imu_sample_updated);
%     predictState(imu_sample_updated);
% 
%     gps_data_ready = false;
%     gps_dt = data.RTK.t - imu_delta_t(i,1)*1e6;
%     gps_index = find(gps_dt<1e3,1,'last');
%     if gps_index_last~=gps_index    %目前都默认gps数据是能用的
%         gps_data_ready = true;
%         gps_index_last = gps_index;
%     end
% 
%     runYawEKFGSF(imu_sample_updated,data.RTK,gps_data_ready,gps_index);
% 
%     controlGpsFusion(gps_data_ready);
% 
%     K4_Display(:,i) = Kfusion4;
%     states_quat_nominal_display(:,i) = states.quat_nominal;
%     states_vel_display(:,i) = states.vel;
%     states_pos_display(:,i) = states.pos;
% %     gps_index_display(i,1) = gps_index;
% %     plot(gps_index)
% %     hold on
% %     delta_ang_var_accum_display(i,:) = delta_angle_var_accum;
% %     delta_vel_var_accum_display(i,:) = delta_vel_var_accum;
% 
% %     delta_P = P_M - P;
% % 
% %     set(h,'ZData',P)
% %     pause(0.01);
% %     set(h_m,'ZData',P_M)
% %     pause(0.01);
% 
% end

%%
figure
% plot(gps_index_display)
plot(imu_delta_t,states_vel_display(1,:))

%%
figure
plot(imu_delta_t,states_pos_display(1,:))
%% 状态校正


%%
    mag_sample.time_us = data.MAG.t;
    mag_sample.X = data.MAG.X*0.003;
    mag_sample.Y = data.MAG.Y*0.003;
    mag_sample.Z = data.MAG.Z*0.003;
    figure
    plot(mag_sample.time_us,mag_sample.X,mag_sample.time_us,mag_sample.Y,mag_sample.time_us,mag_sample.Z)
%%
K4_Display = zeros(23,len_update);
len_gps = length(data.RTK.t);
states_quat_nominal_display=zeros(4,len_update);
states_vel_display=zeros(3,len_update);
states_pos_display=zeros(3,len_update);
gps_index_last = 0;


last_gps_fail_us = imu_delta_t(1,1);

%     K4_Display(:,i) = Kfusion4;
%     states_quat_nominal_display(:,i) = states.quat_nominal;
%     states_vel_display(:,i) = states.vel;
%     states_pos_display(:,i) = states.pos;
