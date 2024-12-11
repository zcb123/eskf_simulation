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

quat_nominal_display = zeros(len_t,4);
vel_display = zeros(len_t,3);
pos_display = zeros(len_t,3);


euler = zeros(len_t,3);
vehicle_at_rest_diaplay = nan(len_t,1);
dTheta_display = zeros(len_t,3);
dTheta = [0 0 0]';
ang_bias_display = zeros(len_t,3);
ang_bias = [0 0 0]';
delta_ang_bias_dispaly = zeros(len_t,3);
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
        controlGpsFusion();        
        controlHeightFusion();
        controlZeroVelocityUpdate(); 
        controlFakePosFusion();

        update_deadreckoning_status();
     
    end
    dTheta_display(i,:) = dTheta;
    ang_bias_display(i,:) = ang_bias;
    delta_ang_bias_dispaly(i,:) = states.delta_ang_bias';
    vehicle_at_rest_diaplay(i,1) = control_status.flags.vehicle_at_rest;
    euler(i,:) = quattoeuler(states.quat_nominal);

    calculateOutputStates(newest_high_rate_imu_sample,updated);
    
   
    quat_nominal_display(i,:) = output_new.quat_nominal';
    vel_display(i,:) = output_new.vel';
    pos_display(i,:) = output_new.pos';

end
%%
figure('Name','dTheta_display')
plot(dTheta_display,'*-')
figure('Name','euler')
plot(euler,'*-')
figure('Name','ang_bias_display')
plot(ang_bias_display,'*-');
figure('Name','delta_ang_bias_dispaly')
plot(delta_ang_bias_dispaly,'*-');
%%
eskf_t = double(data.ESKF.t)/1e6;
figure
plot(eskf_t,data.ESKF.roll,'*-');
hold on
plot(vehicle_t,euler(:,1),'*-')
figure
plot(eskf_t,data.ESKF.pitch,'*-');
hold on
plot(vehicle_t,euler(:,2),'*-')
% figure
% plot(vel_display)
% figure
% plot(pos_display)