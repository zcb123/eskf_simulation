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

reset_setIMUData(0.0025);

reset_eskf();

global control_status; 
control_status.flags.in_air = true;

global accel_lpf_NE;
accel_lpf_NE = zeros(2,1);

%% gps
gps_index_last = 0;
global last_gps_fail_us
last_gps_fail_us = 0;
global NED_origin_initialised gps_alt_ref hgt_sensor_offset gps_prev;
NED_origin_initialised = false;
hgt_sensor_offset = 0;
gps_alt_ref = 0;
gps_prev.fix_type = 1;
%% mag
global mag_yaw_reset_req non_mag_yaw_aiding_running_prev mag_inhibit_yaw_reset_req ...
    is_yaw_fusion_inhibited mag_counter flt_mag_align_start_time mag_declination_gps ...
    mag_decl_cov_reset time_yaw_started ;
    
global mag_lpf mag_bias_observable yaw_angle_observable time_last_mov_3d_mag_suitable;
global saved_mag_bf_variance saved_mag_ef_ne_covmat saved_mag_ef_d_variance;
global last_static_yaw mag_test_ratio;
mag_yaw_reset_req = false;
non_mag_yaw_aiding_running_prev = false;
mag_inhibit_yaw_reset_req = false;
is_yaw_fusion_inhibited = false;
mag_counter = 0;
flt_mag_align_start_time = 0;
mag_declination_gps = nan;
mag_decl_cov_reset = false;
mag_lpf = zeros(3,1);
mag_bias_observable = false;	
yaw_angle_observable = false;
time_yaw_started = 0;
time_last_mov_3d_mag_suitable = 0;
saved_mag_bf_variance = zeros(3,1);
saved_mag_ef_ne_covmat = zeros(2,2);
saved_mag_ef_d_variance = 0;
last_static_yaw = nan;
mag_test_ratio = zeros(3,1);