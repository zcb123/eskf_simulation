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
close all
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

        runYawEKFGSF();

        controlHeightSensorTimeouts();
        
        controlMagFusion();
        %controlOpticalFlowFusion();
        controlGpsFusion();
    %   controlAirDataFusion();
    % 	controlBetaFusion();
    % 	controlDragFusion();
        controlHeightFusion();
    end

    calculateOutputStates(newest_high_rate_imu_sample,updated);
    
    

end
