run("data_preProcess.m");
close all
global params control_status control_status_prev filter_initialised newest_high_rate_imu_sample;
global GNSS GPSYAW;
global P output_new;
global states;
%update default params
params.gps_pos_noise = 0.03;
params.gps_vel_noise = 0.03;
params.accel_noise = 0.35;
params.vdist_sensor_type =GNSS;
params.fusion_mode = bitor(params.fusion_mode,GPSYAW);

len_t = length(vehicle_t);
vehicle_dt = zeros(len_t,1);
vehicle_dt(end,1) = 0.0025;
vehicle_dt(1:end-1,:) = diff(vehicle_t);


% display val

quat_nominal_display = nan(len_t,4);
update_t_nan = nan(len_t,1);
vel_nan_display = nan(len_t,3);
pos_nan_display = nan(len_t,3);
states_vel_fakeUptated_nan_display = nan(len_t,3); 
states_delta_vel_bias_fakeUptaed_nan_display = nan(len_t,3);
states_vel_baro_nan_display = nan(len_t,3);
states_vel_bias_baro_nan_display = nan(len_t,3);

euler_nan = nan(len_t,3);
vehicle_at_rest_diaplay = nan(len_t,1);
dTheta_display = nan(len_t,3);
dTheta = [0 0 0]';
ang_bias_display = nan(len_t,3);
ang_bias = [0 0 0]';
delta_ang_bias_dispaly = nan(len_t,3);

delta_vel_earth_nan_display = nan(len_t,3);
vel_correction_nan_display = nan(len_t,3);
pos_correction_nan_display = nan(len_t,3);
delta_vel_earth = [0 0 0]';

corrected_delta_vel_display = zeros(len_t,3);
delta_P = zeros(23,23);
delta_P_small = nan(12,12,426);
figure('Name','P')
h = surf(delta_P);
shading interp; % 平滑着色
colormap(jet);
colorbar;
xlabel('X 轴');
ylabel('Y 轴');
zlabel('Z 轴');
title('P 动态演示');
global dt_ekf_avg
dt_ekf_avg_nan_display = nan(len_t,1);
clear setSensorData controlGpsYawFusion calculateOutputStates
update_index = 1;
for i = 1:len_t

    updated = setSensorData(i,gyro_filted,acc_filted,vehicle_t,vehicle_dt,data);
    
    if filter_initialised && control_status.flags.in_air
        set_in_air_status(false);   %滤波器初始化成功后这里置false
    end

    if ~filter_initialised
            filter_initialised = initialiseFilter();
            
            if ~filter_initialised
                disp("eskf initialised failed");
                continue;
            end
    end
    euler_tmp = quattoeuler(states.quat_nominal);
    if updated

        predictCovariance();

        P_Cov = P;
        
        predictState();

        dt_ekf_avg_nan_display(i,1) = dt_ekf_avg;
        euler_tmp = quattoeuler(states.quat_nominal);
        controlFusionModes(); 
        runYawEKFGSF();
        controlHeightSensorTimeouts();

        controlMagFusion();
%         delta_P = P-P_Cov;
%         set(h,'ZData',delta_P);
%         pause(0.1);
        delta_P_small(:,:,update_index) = delta_P(1:12,1:12);
        update_index = update_index + 1;
        controlGpsFusion();        
        controlHeightFusion();
        states_vel_baro_nan_display(i,:) = delta_vel';
        states_vel_bias_baro_nan_display(i,:) = delta_vel_bias';

        controlZeroVelocityUpdate();        %这里面没运行 at_rest = false
        controlFakePosFusion();
        states_vel_fakeUptated_nan_display(i,:) = delta_vel';
        states_delta_vel_bias_fakeUptaed_nan_display(i,:) = delta_vel_bias';

        update_deadreckoning_status();
        
%         
    end
    
    calculateOutputStates(newest_high_rate_imu_sample,updated);
    delta_vel_earth_nan_display(i,:) = delta_vel_earth';
    vel_correction_nan_display(i,:) = vel_correction';
    pos_correction_nan_display(i,:) = pos_correction';
%     states.delta_vel_bias';
    if updated

        dTheta_display(i,:) = dTheta;
        ang_bias_display(i,:) = ang_bias;
        delta_ang_bias_dispaly(i,:) = states.delta_ang_bias';
        vehicle_at_rest_diaplay(i,1) = control_status.flags.vehicle_at_rest;
        euler_nan(i,:) = quattoeuler(output_new.quat_nominal);
        quat_nominal_display(i,:) = output_new.quat_nominal';
        vel_nan_display(i,:) = output_new.vel';
        pos_nan_display(i,:) = output_new.pos';
        update_t_nan(i,1) = vehicle_t(i,1);

    end
end
euler = getNonNaN(euler_nan,3);
vel_display = getNonNaN(vel_nan_display,3);
pos_display = getNonNaN(pos_nan_display,3);
update_t = getNonNaN(update_t_nan,1);
%%
% figure
% plot(mag_lpf_display)
% figure
% plot(mag_sample_new_display)
%%
% figure('Name','dTheta_display')
% plot(dTheta_display,'*-')
delta_ang_bias_dispaly = getNonNaN(delta_ang_bias_dispaly,3);
figure('Name','ang_bias_display')
plot(ang_bias_display,'*-');
figure('Name','delta_ang_bias_dispaly')
plot(delta_ang_bias_dispaly,'*-');
%%
dt_ekf_avg_display = getNonNaN(dt_ekf_avg_nan_display,1);
figure
plot(dt_ekf_avg_display)        
%%
states_vel_baro_display = getNonNaN(states_vel_baro_nan_display,3);
states_vel_bias_baro_display = getNonNaN(states_vel_bias_baro_nan_display,3);
figure('Name','states_vel_baro_display')
plot(states_vel_baro_display);
figure('Name','states_vel_bias_baro_display')
plot(states_vel_bias_baro_display)
%%
states_vel_fakeUptated_display = getNonNaN(states_vel_fakeUptated_nan_display,3);
states_delta_vel_bias_fakeUptaed_display = getNonNaN(states_delta_vel_bias_fakeUptaed_nan_display,3);
figure('Name','states_vel_fakeUptated_display')
plot(states_vel_fakeUptated_display);   
figure('Name','states_delta_vel_bias_fakeUptaed_display')
plot(states_delta_vel_bias_fakeUptaed_display)
%%
delta_vel_earth_display = getNonNaN(delta_vel_earth_nan_display,3);
figure('Name','delta_vel_earth_display')
plot(delta_vel_earth_display);
%%
vel_correction_display = getNonNaN(vel_correction_nan_display,3);
pos_correction_display = getNonNaN(pos_correction_nan_display,3);
figure('Name','vel_correction_display')
plot(vel_correction_display);
figure('Name','pos_correction_display')
plot(pos_correction_display)
%% compare euler
eskf_t = double(data.ESKF.t)/1e6;
update_t = getNonNaN(update_t_nan,1);
euler = getNonNaN(euler_nan,3);
figure('Name','euler roll compare')
plot(update_t,euler(:,1),'*-');
hold on
plot(eskf_t,data.ESKF.roll,'*-');
figure('Name','euler pitch compare')
plot(update_t,euler(:,2),'*-');
hold on
plot(eskf_t,data.ESKF.pitch,'*-');
figure('Name','euler yaw compare')
plot(update_t,euler(:,3),'*-');
hold on
plot(eskf_t,data.ESKF.yaw,'*-');
%% compare vel pos
vel_display = getNonNaN(vel_nan_display,3);
eskf_t = double(data.ESKF.t)/1e6;
update_t = getNonNaN(update_t_nan,1);
figure('Name','vN')
plot(update_t,vel_display(:,1),'*-');
hold on
plot(eskf_t,data.ESKF.vN,'*-');
figure('Name','vE')
plot(update_t,vel_display(:,2),'*-');
hold on
plot(eskf_t,data.ESKF.vE,'*-');
figure('Name','vD')
plot(update_t,vel_display(:,3),'*-');
hold on
plot(eskf_t,data.ESKF.vD,'*-');
%%
pos_display = getNonNaN(pos_nan_display,3);
figure('Name','pN')
plot(pos_display(:,1),'*-');
hold on
plot(data.ESKF.pN,'*-');
figure('Name','pE')
plot(pos_display(:,2),'*-');
hold on
plot(data.ESKF.pE,'*-');
figure('Name','pD')
plot(pos_display(:,3),'*-');
hold on
plot(data.ESKF.pD,'*-');