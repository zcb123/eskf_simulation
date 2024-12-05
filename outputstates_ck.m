run("data_preProcess.m");

global params control_status filter_initialised newest_high_rate_imu_sample;
global GNSS GPSYAW;
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
global yaw_delta_ef output_new;

delta_angle_display = zeros(len_t,3);
delta_angle_corr_display = zeros(len_t,3);
yaw_delta_ef_display = zeros(len_t,1);
quat_nominal_display = zeros(len_t,4);
vel_display = zeros(len_t,3);
pos_display = zeros(len_t,3);
output_new_pos_display = zeros(len_t,3);
vel_correction_display = zeros(len_t,3);
pos_correction_display = zeros(len_t,3);
vel_correction = [0 0 0]';
pos_correction = [0 0 0]';
euler = zeros(len_t,3);
dq_display = zeros(len_t,4);
quat_delta_delay_display = zeros(len_t,4);
% running
clear setSensorData calculateOutputStates;
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
%         predictState();


        
    end

    calculateOutputStates(newest_high_rate_imu_sample,updated);
    
    euler(i,:) = quattoeuler(output_new.quat_nominal);
    delta_angle_display(i,:) = delta_angle';
    delta_angle_corr_display(i,:) = delta_angle_corr';
    yaw_delta_ef_display(i,1) = yaw_delta_ef;
    quat_delta_delay_display(i,:) = quat_delta_delay';
    output_new_pos_display(i,:) = output_new_pos';
    quat_nominal_display(i,:) = output_new.quat_nominal';
    vel_display(i,:) = output_new.vel';
    pos_display(i,:) = output_new.pos';
    vel_correction_display(i,:) = vel_correction';
    pos_correction_display(i,:) = pos_correction';

end


%%
% close all
figure
plot(delta_angle_display);
legend('x','y','z')
figure('Name','yaw_delta_ef_display')
plot(yaw_delta_ef_display);
figure('Name','quat_nominal')
plot(quat_nominal_display);
figure('Name','vel')
plot(vel_display);
figure('Name','pos un-updated')
plot(output_new_pos_display);
figure('Name','pos')
plot(pos_display);
figure('Name','vel_correction_display')
plot(vel_correction_display)
figure('Name','pos_correction_display')
plot(pos_correction_display)

figure('Name','euler')
plot(euler);

%%
figure
plot(quat_delta_delay_display)
%%
figure
plot(data.ESKF.pN);
hold on
plot(data.ESKF.pE);
hold on
plot(data.ESKF.pD);
%%
figure
plot(data.ESKF.vN);
hold on
plot(data.ESKF.vE);
hold on
plot(data.ESKF.vD);


