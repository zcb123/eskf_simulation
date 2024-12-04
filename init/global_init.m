
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



%%
global params control_status fault_status;

params.imu_pos_body = single([1 0 0]');
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

control_status.flags.tilt_align = 1;
control_status.flags.yaw_align = 1;
control_status.flags.mag_3D = 1;
control_status.flags.wind = 1;
control_status.flags.in_air = 1;
control_status.flags.gps = 1;
control_status.flags.gps_yaw = 1;
control_status.flags.gps_yaw_fault = 1;
control_status.flags.gps_hgt = 1;
control_status.flags.baro_hgt = 1;
control_status.flags.rng_hgt = 1;
control_status.flags.mag_field_disturbed = 1;
control_status.flags.vehicle_at_rest = 1;
control_status.flags.mag_aligned_in_flight = 1;
control_status.flags.gnd_effect = 1;
control_status.flags.fixed_wing = 1;
control_status.flags.opt_flow = 1;
control_status.flags.ev_pos = 1;
control_status.flags.ev_vel = 1;
control_status.flags.fuse_beta = 1;
control_status.flags.fuse_aspd = 1;

% control_status.value = sum(control_status.flags.)

fault_status.flags.bad_vel_N = logical(true);
fault_status.flags.bad_hdg = false;
fault_status.flags.bad_acc_vertical = false;


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


global output_new;
output_new = struct('time_us',uint64(0),'quat_nominal',single([1 0 0 0]'),'vel',single([0 0 0]'),'pos',single([0 0 0]'));
% output_buffer = ring_buffer(obs_buffer_length);

global filter_initialised is_first_imu_sample
is_first_imu_sample = true;
filter_initialised = false;
%% IMU 相关
global initialised
initialised = false;

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
global gps_checks_passed NED_origin_initialised gps_alt_ref hgt_sensor_offset gps_prev time_last_on_ground_us gps_yaw_offset;
gps_checks_passed = false;
NED_origin_initialised = false;
hgt_sensor_offset = 0;
gps_alt_ref = 0;
gps_prev.fix_type = 1;
time_last_on_ground_us = 0;
gps_yaw_offset = 0;


clear controlGpsYawFusion        

%% MAG 相关
global mag_buffer time_last_mag
mag_buffer = ring_buffer(obs_buffer_length);
time_last_mag = 0;
%% BARO 相关
global baro_buffer time_last_baro baro_hgt_offset baro_counter
baro_buffer = ring_buffer(obs_buffer_length);
time_last_baro = 0;
baro_hgt_offset = 0;
baro_counter = 0;
%% sensor lpf 
global accel_lpf gyro_lpf mag_lpf;
accel_lpf = AlphaFilter(0.1,0);
gyro_lpf = AlphaFilter(0.1,0);
mag_lpf = AlphaFilter(0.1,0);