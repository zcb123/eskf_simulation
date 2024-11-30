clear setIMUData;

clear predictState predictCovariance predictCovariance_Matrix ...
    fuseVelocityWithLevelArm fuseVelocityWithLevelArm_Matrix ;   

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

global NED_origin_initialised;
NED_origin_initialised = false;




reset_setIMUData(0.0025);
gps_index_last = 0;