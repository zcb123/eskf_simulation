clear setIMUData;

clear predictState predictCovariance predictCovariance_Matrix ...
    fuseVelocityWithLevelArm fuseVelocityWithLevelArm_Matrix ;   

clear alphaFilter Copy_of_alphaFilter;

global filter_initialised is_first_imu_sample
filter_initialised = false;
is_first_imu_sample = true;
len_update = length(imu_delta_t);

init_states();

run("P_init.m");

clear yawEstimator;
global yawEstimator;
yawEstimator = EKFGSF_YAW();

global last_gps_fail_us
last_gps_fail_us = 0;

global NED_origin_initialised gps_alt_ref hgt_sensor_offset gps_prev;
NED_origin_initialised = false;
hgt_sensor_offset = 0;
gps_alt_ref = 0;
gps_prev.fix_type = 1;

reset_setIMUData(0.0025);
gps_index_last = 0;

reset_eskf();

global control_status; 
control_status.flags.in_air = true;

