
global CONSTANTS_ONE_G CONSTANTS_RADIUS_OF_EARTH FLT_EPSILON;
CONSTANTS_ONE_G = single(9.80665);
FLT_EPSILON = 1.192e-7;
CONSTANTS_RADIUS_OF_EARTH = 6371000;

%mag fusion type
global AUTO HEADING MAG_3D UNUSED INDOOR NONE;
AUTO = 0;
HEADING = 1;
MAG_3D = 2;
UNUSED = 3;
INDOOR = 4;
NONE = 5;
%0:BARO 1:GNSS 2:RANGE 3:EV
%vdist_sensor_type 
global BARO GNSS RANGE EV UNKNOWN
BARO     = 0;	%/< Use baro height
GNSS     = 1;	%/< Use GPS height
RANGE    = 2;	%/< Use range finder height
EV       = 3;   %/< Use external vision
UNKNOWN  = 4;    %/< Unknown

%fusion mode
global GPS OF INHIBIT_ACC_BIAS EVPOS EVYAW DRAG ROTATE_EV GPSYAW EVVEL
GPS  =  bitshift(1,0);		%/< set to true to use GPS data
OF    =  bitshift(1,1);	%/< set to true to use optical flow data
INHIBIT_ACC_BIAS  = bitshift(1,2);	%/< set to true to inhibit estimation of accelerometer delta velocity bias
EVPOS	= bitshift(1,3);		%/< set to true to use external vision position data
EVYAW   = bitshift(1,4);		%/< set to true to use external vision quaternion data for yaw
DRAG    = bitshift(1,5);		%/< set to true to use the multi-rotor drag model to estimate wind
ROTATE_EV  = bitshift(1,6);		%/< set to true to if the EV observations are in a non NED reference frame and need to be rotated before being used
GPSYAW  = bitshift(1,7);		%/< set to true to use GPS yaw data if available
EVVEL   = bitshift(1,8);

global USE_GEO_DECL SAVE_GEO_DECL FUSE_DECL
USE_GEO_DECL = bitshift(1,0);
SAVE_GEO_DECL = bitshift(1,1);
FUSE_DECL = bitshift(1,2);
%%
global params control_status fault_status;

params.imu_pos_body = single([0 0 0]');
params.gps_pos_body = single([0 0 0]');
params.vel_Tau = single(0.25);
params.pos_Tau = single(0.25);
params.filter_update_interval_us = single(8000);
params.gyro_noise = single(1.5e-2);
params.gyro_bias_p_noise = single(1e-3);
params.accel_noise = single(3.5e-1);
params.accel_bias_p_noise = single(3e-3);
params.wind_vel_p_noise = single(1e-1);
params.wind_vel_p_noise_scaler = single(0.5);
params.acc_bias_learn_tc = single(0.5);
params.acc_bias_learn_gyr_lim = single(3);
params.acc_bias_learn_acc_lim = single(25);
params.fusion_mode = bitor(GPS,GPSYAW);
params.mage_p_noise = single(1e-3);
params.magb_p_noise = single(1e-4);
params.initial_wind_uncertainty = single(1);
params.pos_noaid_noise = single(10);
params.gps_pos_innov_gate = single(5);
params.gps_vel_innov_gate = single(5);
params.gps_pos_noise = single(0.5);
params.gps_vel_noise = single(0.3);
params.switch_on_gyro_bias = single(0.1);
params.switch_on_accel_bias = single(0.2);
params.mag_noise = single(5e-2);
params.initial_wind_uncertainty = single(1);
params.initial_tilt_err = single(0.1);
params.gps_yaw_offset = single(180);                                %出货机天线航向偏置180，碳管机偏置90
params.gps_heading_noise = single(0.1);
params.heading_innov_gate = single(2.6);
params.reset_timeout_max = 7000000;         %微秒
params.EKFGSF_reset_delay = 1000000;
params.EKFGSF_yaw_err_max = 0.262;
params.mag_fusion_type = 0; %AUTO=0;HEADING=1,MAG_3D=2,UNUSED=3,INDOOR=4,NONE=5
params.req_vacc = 8.0;
params.baro_innov_gate = 5;
params.baro_noise = 3.5;
params.EKFGSF_reset_count_limit = 3;
params.mag_declination_deg = 0;
params.mag_acc_gate = 0.5;
params.mag_yaw_rate_gate = 0.25;
params.check_mag_strength = 0;
params.mag_declination_source = 7;
params.mag_heading_noise = 3e-1;
params.vdist_sensor_type = 1; %0:BARO 1:GNSS 2:RANGE 3:EV
params.sensor_interval_max_ms = 10;
params.acc_bias_lim = 0.4;
params.synthesize_mag_z = 0;
params.gnd_effect_deadzone = 5;

params.max_correction_airspeed = 20;
params.static_pressure_coef_xp = 0;
params.static_pressure_coef_xn = 0;
params.static_pressure_coef_yp = 0;
params.static_pressure_coef_yn = 0;
params.static_pressure_coef_z = 0;
params.max_correction_airspeed = 20;
params.mag_delay_ms = 0;		%/< magnetometer measurement delay relative to the IMU (mSec)
params.baro_delay_ms = 0;		%/< barometer height measurement delay relative to the IMU (mSec)
params.gps_delay_ms = 110;		%/< GPS measurement delay relative to the IMU (mSec)
params.airspeed_delay_ms = 100;	%/< airspeed measurement delay relative to the IMU (mSec)
params.flow_delay_ms = 5;		%/< optical flow measurement delay relative to the IMU (mSec) - this is to the middle of the optical flow integration interval
params.range_delay_ms = 5;		%/< range finder measurement delay relative to the IMU (mSec)
params.ev_delay_ms = 175;		%/< off-board vision measurement delay relative to the IMU (mSec)
params.auxvel_delay_ms = 5;		%/< auxiliary velocity measurement delay relative to the IMU (mSec)
params.arsp_thr = 2;
params.range_aid = 0;           %allow switching primary height source to range finder if certain conditions are met
params.bad_acc_reset_delay_us = 500000;
params.vert_innov_test_lim = 3;
params.vert_innov_test_min = 1;

control_status.flags.tilt_align = 1;
control_status.flags.yaw_align = 1;
control_status.flags.mag_3D = 1;
control_status.flags.wind = false;
control_status.flags.in_air = 1;
control_status.flags.gps = 1;
control_status.flags.gps_yaw = 1;
control_status.flags.gps_yaw_fault = false;
control_status.flags.gps_hgt = 1;
control_status.flags.baro_hgt = 1;
control_status.flags.rng_hgt = false;
control_status.flags.mag_field_disturbed = 1;
control_status.flags.vehicle_at_rest = 1;
control_status.flags.mag_aligned_in_flight = 1;
control_status.flags.mag_fault = false;
control_status.flags.gnd_effect = 1;
control_status.flags.fixed_wing = false;
control_status.flags.opt_flow = false;
control_status.flags.ev_pos = false;
control_status.flags.ev_vel = false;
control_status.flags.ev_yaw = false;
control_status.flags.fuse_beta = false;
control_status.flags.fuse_aspd = false;

global control_status_prev;
% control_status.value = sum(control_status.flags.)

fault_status.flags.bad_vel_N = logical(true);
fault_status.flags.bad_hdg = false;
fault_status.flags.bad_acc_vertical = false;
fault_status.flags.bad_pos_D = false;

global states dt_imu_avg  dt_ekf_avg R_to_earth;
states = struct('quat_nominal',single([1 0 0 0]'),...
                        'vel',single([0 0 0]'),...
                        'pos',single([0 0 0]'),...
                        'delta_ang_bias',single([0 0 0]'),...
                        'delta_vel_bias',single([0 0 0]'),...
                        'mag_I',single([0 0 0]'),...
                        'mag_B',single([0 0 0]'),...
                        'wind_vel',single([0 0]'));
dt_imu_avg = 0.004;
dt_ekf_avg = 0.008;
R_to_earth = zeros(3,3);
global k_vel_id k_vel_bias_id
k_vel_id = 4;
k_vel_bias_id = 13;
%% ALL
global state_reset_status;
state_reset_status = struct('velNE_counter',0,'velD_counter',0,'posNE_counter',0,'posD_counter',0,'quat_counter',0, ...
    'velD_change',0,'posNE_change',[0 0]','posD_change',0,'quat_change',[1 0 0 0]');
global vel_imu_rel_body_ned
vel_imu_rel_body_ned = [0 0 0]';
global time_acc_bias_check
time_acc_bias_check = 0;
global gps_vel_innov ev_vel_innov gps_pos_innov baro_hgt_innov rng_hgt_innov ev_pos_innov;
gps_vel_innov = zeros(3,1);
ev_vel_innov = zeros(3,1);
gps_pos_innov = zeros(3,1);
baro_hgt_innov = 0;
rng_hgt_innov = 0;
ev_pos_innov = zeros(3,1);

global obs_buffer_length;
obs_buffer_length = 0;

global output_new yaw_delta_ef delta_angle_corr R_to_earth_now;
output_new = struct('time_us',uint64(0),'quat_nominal',single([1 0 0 0]'),'vel',single([0 0 0]'),'pos',single([0 0 0]'));
yaw_delta_ef = 0;
delta_angle_corr = 0;
R_to_earth_now = zeros(3,3);

global filter_initialised is_first_imu_sample time_last_imu
is_first_imu_sample = true;
filter_initialised = false;
time_last_imu = 0;

global time_last_in_air;
time_last_in_air = 0;

%%  predictCovariance
global ang_rate_magnitude_filt accel_magnitude_filt accel_vec_filt accel_bias_inhibit;
ang_rate_magnitude_filt = 0;
accel_magnitude_filt = 0;
accel_vec_filt = 0;
accel_bias_inhibit = logical([false false false]);

global delta_angle_var_accum delta_vel_var_accum delta_angle_bias_var_accum delta_vel_bias_var_accum;
delta_angle_var_accum = 0;
delta_vel_var_accum = 0;
delta_angle_bias_var_accum = 0;
delta_vel_bias_var_accum = 0;
%% predictStates
global ang_rate_delayed_raw
ang_rate_delayed_raw = 0;


global accel_lpf_NE;
accel_lpf_NE = zeros(2,1);
%% IMU 相关
global initialised
initialised = false;

global imu_down_sampler;
imu_down_sampler = ImuDownSampler(params.filter_update_interval_us);

global min_obs_interval_us
min_obs_interval_us = 0;

global vibe_metrics delta_ang_prev delta_vel_prev time_last_move_detect_us; 
vibe_metrics = [0 0 0]';
delta_ang_prev = [0 0 0]';
delta_vel_prev = [0 0 0]';
time_last_move_detect_us = 0;

%% GPS 相关
global gps_buffer time_last_gps pos_ref
time_last_gps = 0;
gps_buffer = ring_buffer(obs_buffer_length);
pos_ref = MapProjection();

global last_gps_fail_us
last_gps_fail_us = 0;
global gps_checks_passed NED_origin_initialised gps_alt_ref hgt_sensor_offset gps_prev time_last_on_ground_us gps_yaw_offset gps_acc_changed;
gps_checks_passed = false;
NED_origin_initialised = false;
hgt_sensor_offset = 0;
gps_alt_ref = 0;
gps_prev.fix_type = 1;
time_last_on_ground_us = 0;
gps_yaw_offset = 0;
gps_acc_changed = false;

global gps_intermittent time_prev_gps_us gps_hgt_accurate;
gps_intermittent = false;
time_prev_gps_us = 0;
gps_hgt_accurate = false;

global gps_sample_delayed gps_data_ready
gps_sample_delayed = struct('time_us',0,...
                            'pos',[0 0]',...	%/< 水平位置(相对于home点的北东向为正)
				            'hgt',	0,...	%/< 海拔高度(向上为正)
			                'vel',[0 0 0]',...		%/< gps速度(北东地)
		                    'yaw', 0 ,...    %/< 板卡融合出的航向
				            'hacc',	0,...	%/< 水平位置标准差
				            'vacc',	0,...	%/< 海拔高度标准差
				            'sacc',0,...		%/< 速度标准差
		                    'fix_type',0);
gps_data_ready = false;

global gps_pos_innov_var gps_pos_test_ratio gps_vel_test_ratio
gps_vel_test_ratio = [0 0 0]';
gps_pos_innov_var = [0 0 0]';
gps_pos_test_ratio = [0 0]';





global time_last_hgt_fuse time_last_hor_pos_fuse time_last_hor_vel_fuse time_last_hagl_fuse time_last_flow_terrain_fuse time_last_of_fuse;

time_last_hgt_fuse = 0;
time_last_hor_pos_fuse = 0;
time_last_hor_vel_fuse = 0;
time_last_hagl_fuse = 0;
time_last_flow_terrain_fuse = 0;
time_last_of_fuse = 0;


clear controlGpsYawFusion yaw_signed_test_ratio_lpf       

%% MAG 相关
global mag_buffer time_last_mag
mag_buffer = ring_buffer(obs_buffer_length);
time_last_mag = 0;

global mag_yaw_reset_req non_mag_yaw_aiding_running_prev mag_inhibit_yaw_reset_req ...
    is_yaw_fusion_inhibited mag_counter flt_mag_align_start_time mag_declination_gps ...
    mag_decl_cov_reset time_yaw_started;
    
global mag_bias_observable yaw_angle_observable time_last_mov_3d_mag_suitable yaw_rate_lpf_ef;
global saved_mag_bf_variance saved_mag_ef_ne_covmat saved_mag_ef_d_variance;
global last_static_yaw mag_test_ratio;

global mag_strength_gps;
mag_yaw_reset_req = false;
non_mag_yaw_aiding_running_prev = false;
mag_inhibit_yaw_reset_req = false;
is_yaw_fusion_inhibited = false;
mag_counter = 0;
flt_mag_align_start_time = 0;
mag_declination_gps = nan;
mag_decl_cov_reset = false;
mag_bias_observable = false;	
yaw_angle_observable = false;
time_yaw_started = 0;
time_last_mov_3d_mag_suitable = 0;
yaw_rate_lpf_ef = 0;
saved_mag_bf_variance = zeros(3,1);
saved_mag_ef_ne_covmat = zeros(2,2);
saved_mag_ef_d_variance = 0;
last_static_yaw = nan;
mag_test_ratio = zeros(3,1);
mag_strength_gps = nan;
global USE_GEO_DECL
USE_GEO_DECL = bitshift(1,0);

%% BARO 相关
global baro_buffer time_last_baro baro_hgt_offset baro_counter terrain_vpos
baro_buffer = ring_buffer(obs_buffer_length);
time_last_baro = 0;
baro_hgt_offset = 0;
baro_counter = 0;
terrain_vpos = 0;
global baro_hgt_intermittent delta_time_baro_us baro_hgt_faulty;
baro_hgt_intermittent = false;
delta_time_baro_us = 0;
baro_hgt_faulty = false;


global baro_b_est;
baro_b_est = BaroBiasEstimator();

%% sensor lpf 
global accel_lpf gyro_lpf mag_lpf;
accel_lpf = AlphaFilter(0.1,0);
gyro_lpf = AlphaFilter(0.1,0);
mag_lpf = AlphaFilter(0.1,0);

%% EKFGSF
global yawEstimator
yawEstimator = EKFGSF_YAW();
%% 速度位置
global vert_pos_fuse_attempt_time_us vert_vel_fuse_time_us vert_pos_innov_ratio vert_vel_innov_ratio;
vert_pos_fuse_attempt_time_us = 0;
vert_vel_fuse_time_us = 0;
vert_pos_innov_ratio = 0;
vert_vel_innov_ratio = 0;
global clip_counter time_bad_vert_accel time_good_vert_accel;
clip_counter = 0;
time_bad_vert_accel = 0;
time_good_vert_accel = 0;

global time_last_zero_velocity_fuse
time_last_zero_velocity_fuse = 0;

