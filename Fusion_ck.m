run("data_preProcess.m");

global params control_status control_status_prev filter_initialised newest_high_rate_imu_sample;
global GNSS GPSYAW;
global P output_new;

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

% figure('Name','P')
% h = surf(P);
% shading interp; % 平滑着色
% colormap(jet);
% colorbar;
% xlabel('X 轴');
% ylabel('Y 轴');
% zlabel('Z 轴');
% title('P 动态演示');
global time_last_ver_vel_fuse gps_hgt_accurate time_last_zero_velocity_fuse;
global states yaw_test_ratio;
quat_nominal_display = zeros(len_t,4);
vel_display = zeros(len_t,3);
pos_display = zeros(len_t,3);

time_last_ver_vel_fuse_display = zeros(len_t,1);
baro_hgt_display = zeros(len_t,1);
gps_yaw_display = zeros(len_t,1);
gps_hgt_display = zeros(len_t,1);
gps_hgt_accurate_display = zeros(len_t,1);
time_last_zero_velocity_fuse_display = zeros(len_t,1);
dq_dt_display = zeros(len_t,4);
dq_dt = [1 0 0 0]';
delta_ang_bias_display = zeros(len_t,3);
Kfusion_yaw_display = zeros(len_t,23);
Kfusion4_display = zeros(len_t,23);
Kfusion5_display = zeros(len_t,23);
Kfusion6_display = zeros(len_t,23);
Kfusion =0 ;
Kfusion4 = zeros(1,23);
Kfusion5 = zeros(1,23);
Kfusion6 = zeros(1,23);

innovation_display = zeros(len_t,1);
innovation = 0;
yaw_test_ratio_display = zeros(len_t,1);
fuse_time_display = zeros(len_t,2);
fuse_time_gps = 0;
fuse_time_imu = 0;
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
        

        control_status_prev = control_status;

        controlFusionModes();

        runYawEKFGSF();

        controlHeightSensorTimeouts();
        
        controlMagFusion();
%                     
        controlGpsFusion();
%                     
        controlHeightFusion();

        controlZeroVelocityUpdate();

        
%         set(h,'ZData',P)
%         pause(0.01);
        
    end
    
    calculateOutputStates(newest_high_rate_imu_sample,updated);

    Kfusion_yaw_display(i,:) = Kfusion;
    Kfusion4_display(i,:) = Kfusion4';
    Kfusion5_display(i,:) = Kfusion5';
    Kfusion6_display(i,:) = Kfusion6';
    innovation_display(i,:) = innovation;
    dq_dt_display(i,:) =dq_dt;
    delta_ang_bias_display(i,:) = states.delta_ang_bias';
    yaw_test_ratio_display(i,1) = yaw_test_ratio;
    fuse_time_display(i,:) = [fuse_time_gps fuse_time_imu];


    time_last_ver_vel_fuse_display(i,1) = time_last_ver_vel_fuse;

    baro_hgt_display(i,1) = control_status.flags.baro_hgt;
    gps_yaw_display(i,1) = control_status.flags.gps_yaw;
    gps_hgt_display(i,1) = control_status.flags.gps_hgt;

    gps_hgt_accurate_display(i,1) = gps_hgt_accurate;
    time_last_zero_velocity_fuse_display(i,1) = time_last_zero_velocity_fuse;


    quat_nominal_display(i,:) = output_new.quat_nominal';
    vel_display(i,:) = output_new.vel';
    pos_display(i,:) = output_new.pos';


end
%%

% figure('Name','P')
% h = surf(P);
% shading interp; % 平滑着色
% colormap(jet);
% colorbar;
% xlabel('X 轴');
% ylabel('Y 轴');
% zlabel('Z 轴');
% title('P 动态演示');
%%

figure
plot(fuse_time_display)
%%
figure
plot(fuse_time_display(:,2) - fuse_time_display(:,1))
%%
figure
plot(Kfusion_yaw_display)
figure
plot(innovation_display)
% figure
% plot(Kfusion4_display(:,11));
% figure
% plot(Kfusion6_display(:,11));
% figure
% plot(Kfusion6_display(:,11));
figure('Name','test_ratio yaw')
plot(yaw_test_ratio_display)
hold on 
plot(data.ESTU.mR);
%%
figure
plot(dq_dt_display)
figure
plot(delta_ang_bias_display)
%%
figure('Name','quat_nominal_display')
plot(quat_nominal_display);
figure('Name','vel_display')
plot(vel_display);
figure('Name','pos_display')
plot(pos_display);

% figure
% plot(data.PCMD.q1);
% hold on
% plot(data.PCMD.q2);
% hold on
% plot(data.PCMD.q3);
% hold on
% plot(data.PCMD.q4);
% hold on
% figure
% plot(data.ESKF.vN);
% hold on
% plot(data.ESKF.vE);
% hold on
% plot(data.ESKF.vD);
% hold on
% figure
% plot(data.ESKF.pN);
% hold on
% plot(data.ESKF.pE);
% hold on
% plot(data.ESKF.pD);
% hold on

%%
% figure
% plot(time_last_ver_vel_fuse_display);
% figure('Name','gps_yaw');
% plot(gps_yaw_display);
%%
% figure('Name','baro_hgt_display');
% plot(baro_hgt_display)
% figure('Name','gps_hgt_display');
% plot(gps_hgt_display)
% figure('Name','gps_hgt_accurate_display');
% plot(gps_hgt_accurate_display)
% figure('Name','time_last_zero_velocity_fuse_display');
% plot(time_last_zero_velocity_fuse_display)