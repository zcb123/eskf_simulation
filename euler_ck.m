run("data_preProcess.m");

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
global mag_lpf;
quat_nominal_display = nan(len_t,4);
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
figure('Name','P')
h = surf(P);
shading interp; % 平滑着色
colormap(jet);
colorbar;
xlabel('X 轴');
ylabel('Y 轴');
zlabel('Z 轴');
title('P 动态演示');

clear setSensorData controlGpsYawFusion calculateOutputStates

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
        predictState();
        euler_tmp = quattoeuler(states.quat_nominal);
        controlFusionModes();  
        
        runYawEKFGSF();

        controlHeightSensorTimeouts();
        controlMagFusion();
        controlGpsFusion();        
        controlHeightFusion();
        states_vel_baro_nan_display(i,:) = delta_vel';
        states_vel_bias_baro_nan_display(i,:) = delta_vel_bias';
        controlZeroVelocityUpdate(); 
        controlFakePosFusion();
        states_vel_fakeUptated_nan_display(i,:) = delta_vel';
        states_delta_vel_bias_fakeUptaed_nan_display(i,:) = delta_vel_bias';
        update_deadreckoning_status();
        
%         set(h,'ZData',P);
%         pause(0.1);
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


    end
end
euler = getNonNaN(euler_nan,3);
vel_display = getNonNaN(vel_nan_display,3);
pos_display = getNonNaN(pos_nan_display,3);
%%
% figure
% plot(mag_lpf_display)
% figure
% plot(mag_sample_new_display)
%%
% figure('Name','dTheta_display')
% plot(dTheta_display,'*-')
euler = getNonNaN(euler_nan,3);
figure('Name','euler roll compare')
plot(euler(:,1),'*-');
hold on
plot(data.ESKF.roll,'*-');
figure('Name','euler pitch compare')
plot(euler(:,2),'*-');
hold on
plot(data.ESKF.pitch,'*-');
figure('Name','euler yaw compare')
plot(euler(:,3),'*-');
hold on
plot(data.ESKF.yaw,'*-');
% figure('Name','ang_bias_display')
% plot(ang_bias_display,'*-');
% figure('Name','delta_ang_bias_dispaly')
% plot(delta_ang_bias_dispaly,'*-');
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
%%
vel_display = getNonNaN(vel_nan_display,3);

figure('Name','vN')
plot(vel_display(:,1),'*-');
hold on
plot(data.ESKF.vN,'*-');
figure('Name','vE')
plot(vel_display(:,2),'*-');
hold on
plot(data.ESKF.vE,'*-');
figure('Name','vD')
plot(vel_display(:,3),'*-');
hold on
plot(data.ESKF.vD,'*-');
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