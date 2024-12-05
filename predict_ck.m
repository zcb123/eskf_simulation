run("data_preProcess.m");

global params control_status filter_initialised newest_high_rate_imu_sample;
global GNSS GPSYAW;
global P;
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
figure('Name','P')
h = surf(P);
shading interp; % 平滑着色
colormap(jet);
colorbar;
xlabel('X 轴');
ylabel('Y 轴');
zlabel('Z 轴');
title('P 动态演示');

global delta_angle_var_accum delta_vel_var_accum delta_angle_bias_var_accum delta_vel_bias_var_accum;

delta_angle_var_accum_display = zeros(len_t,3);
delta_vel_var_accum_display = zeros(len_t,3);
delta_angle_bias_var_accum_display = zeros(len_t,3);
delta_vel_bias_var_accum_display = zeros(len_t,3);

global accel_lpf_NE;
accel_lpf_NE_display = zeros(len_t,2);

global states;
quat_display = zeros(len_t,4);
vel_display = zeros(len_t,3);
pos_display = zeros(len_t,3);
% running
clear setSensorData calculateOutputStates predictCovariance;
for i = 1:len_t

    updated = setSensorData(i,gyro_filted,acc_filted,vehicle_t,vehicle_dt,data);

    if filter_initialised && control_status.flags.in_air
        set_in_air_status(false);
    end

    if ~filter_initialised
            filter_initialised = initialiseFilter();
            if ~filter_initialised
                disp("eskf initialised failed");
                continue;
            end
    end

    if updated
        predictCovariance();
        predictState();
        
        controlFusionModes();
        
        delta_angle_var_accum_display(i,:) = delta_angle_var_accum';
        delta_vel_var_accum_display(i,:) = delta_vel_var_accum';
        delta_angle_bias_var_accum_display(i,:) = delta_angle_bias_var_accum';
        delta_vel_bias_var_accum_display(i,:) = delta_vel_bias_var_accum';

        accel_lpf_NE_display(i,:) = accel_lpf_NE';
        quat_display(i,:) = states.quat_nominal';
        vel_display(i,:) = states.vel';
        pos_display(i,:) = states.pos';
%         set(h,'ZData',P);
%         pause(0.01);

    end

    calculateOutputStates(newest_high_rate_imu_sample,updated);
    
    

end

close all
% figure('Name','delta_angle_var_accum_display')
% plot(delta_angle_var_accum_display);
% figure('Name','delta_vel_var_accum_display')
% plot(delta_vel_var_accum_display);
% figure('Name','delta_angle_bias_var_accum_display')
% plot(delta_angle_bias_var_accum_display);
% figure('Name','delta_vel_bias_var_accum_display')
% plot(delta_vel_bias_var_accum_display);
figure
plot(accel_lpf_NE_display);
figure
plot(quat_display);
figure
plot(vel_display);
figure
plot(pos_display);