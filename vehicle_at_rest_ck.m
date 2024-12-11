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
vel_display = nan(len_t,3);
pos_display = nan(len_t,3);


euler_nan = nan(len_t,3);
vehicle_at_rest_diaplay = nan(len_t,1);
dTheta_display = nan(len_t,3);
dTheta = [0 0 0]';
ang_bias_display = nan(len_t,3);
ang_bias = [0 0 0]';
delta_ang_bias_dispaly = nan(len_t,3);

mag_lpf_display = zeros(len_t,3);
mag_sample_new_display = zeros(len_t,3);
mag_sample_new = zeros(1,3);

clear setSensorData controlGpsYawFusion
for i = 1:len_t

    updated = setSensorData(i,gyro_filted,acc_filted,vehicle_t,vehicle_dt,data);
    mag_sample_new_display(i,:) = mag_sample_new';
    if filter_initialised && control_status.flags.in_air
        set_in_air_status(false);
    end

    if ~filter_initialised
            filter_initialised = initialiseFilter();
            mag_lpf_display(i,:) = mag_lpf.getState()';
            if ~filter_initialised
                disp("eskf initialised failed");
                continue;
            end
    end
    euler_tmp = quattoeuler(output_new.quat_nominal);
    if updated

        predictCovariance();
        predictState();
        
        controlFusionModes();  

        runYawEKFGSF();

        controlHeightSensorTimeouts();
        controlMagFusion();
        controlGpsFusion();        
        controlHeightFusion();
        controlZeroVelocityUpdate(); 
        controlFakePosFusion();
        update_deadreckoning_status();
        

        dTheta_display(i,:) = dTheta;
        ang_bias_display(i,:) = ang_bias;
        delta_ang_bias_dispaly(i,:) = states.delta_ang_bias';
        vehicle_at_rest_diaplay(i,1) = control_status.flags.vehicle_at_rest;
        euler_nan(i,:) = quattoeuler(states.quat_nominal);
        quat_nominal_display(i,:) = output_new.quat_nominal';
        vel_display(i,:) = output_new.vel';
        pos_display(i,:) = output_new.pos';

    end
    
    calculateOutputStates(newest_high_rate_imu_sample,updated);
    
 
end
euler = getNonNaN(euler_nan,3);
%%
figure
plot(mag_lpf_display)
figure
plot(mag_sample_new_display)
%%
% figure('Name','dTheta_display')
% plot(dTheta_display,'*-')
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
% figure
% plot(vel_display)
% figure
% plot(pos_display)