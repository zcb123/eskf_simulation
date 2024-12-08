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



euler = zeros(len_t,3);
vehicle_at_rest_diaplay = nan(len_t,1);
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
   
        
    end

    vehicle_at_rest_diaplay(i,1) = control_status.flags.vehicle_at_rest;
    euler(i,:) = quattoeuler(states.quat_nominal);

    calculateOutputStates(newest_high_rate_imu_sample,updated);
    
   


end
%%
figure
plot(vehicle_at_rest_diaplay)