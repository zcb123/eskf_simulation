

foldername = 'C:\Users\89565\Desktop\自研飞控\TestData\十月份数据\1030\';
filename = 'N447_2024-10-30_17-45-26.log';
data = Parse_Data(foldername,filename);

%% 识别解锁标志
rc_t = double(data.RC.t)/1e6;
rc_t = rc_t - rc_t(1,1);
rc1 = data.RC.rc1;
rc2 = data.RC.rc2;
rc3 = data.RC.rc3;
rc4 = data.RC.rc4;
rc5 = data.RC.rc5;
rc6 = data.RC.rc6;
rc7 = data.RC.rc7;
rc8 = data.RC.rc8;
rc_lost = data.RC.lost;
figure
plot(rc_t,rc1,rc_t,rc2,rc_t,rc3,rc_t,rc4);
legend('rc1','rc2','rc3','rc4');
title('rc data');

% control_param.x_p = data.PARM.Value(28);
% control_param.y_p = data.PARM.Value(34);
% control_param.z_p = data.PARM.Value(22);
% climb_rate 54


simin.time = rc_t;
simin.signals.values = [rc1 rc2 rc3 rc4 rc5 rc6 rc7 rc8];

imu_t = double(data.IMU0.t)/1e6;
imu_t = imu_t - imu_t(1,1);
% 赋值的时候顺便把坐标系也转了
acc_y = data.IMU0.AX;
acc_x = data.IMU0.AY;
acc_z = -data.IMU0.AZ;
gyro_x = data.IMU0.GY;
gyro_y = data.IMU0.GX;
gyro_z = -data.IMU0.GZ;
imuData.time = imu_t;
imuData.signals.values = [acc_x acc_y acc_z gyro_x gyro_y gyro_z];

dt = mean(diff(imu_t));
sim_time = imu_t(end,1);

mag_t = double(data.MAG.t)/1e6;
mag_t = mag_t - mag_t(1,1);                 %时间有可能不是从0开始增加
mag_x = double(data.MAG.X);
mag_y = double(data.MAG.Y);
mag_z = double(data.MAG.Z);
magData.time = mag_t;
magData.signals.values = [mag_t mag_x mag_y mag_z];

baro_t = double(data.BAR0.t)/1e6;
baro_t = baro_t - baro_t(1,1);
temp = data.BAR0.Temp;
pressure = data.BAR0.Pressure;
hight = data.BAR0.Hight;
baroData.time = baro_t;
baroData.signals.values = [baro_t temp pressure hight];

gps_t = double(data.RTK.t)/1e6;
gps_t = gps_t - gps_t(1,1);
lon = data.RTK.lon;
lat = data.RTK.lat;
alt = data.RTK.alt;
vel_N_m = [data.RTK.vN data.RTK.vE data.RTK.vD];
yaw = data.RTK.hding;
numSV = data.RTK.spd;
fixtype = data.RTK.fix;
pDop = data.RTK.pdop;
hAcc = data.RTK.hdop;
startNum = data.RTK.svs;        %卫星数
gpsData.time = gps_t-gps_t(1,1);
gpsData.signals.values = [lon lat alt vel_N_m yaw numSV fixtype pDop hAcc startNum];

eskf_t = double(data.ESKF.t)/1e6;
eskf_roll = data.ESKF.roll;
eskf_pitch = data.ESKF.pitch;
eskf_yaw = data.ESKF.yaw;
eskf_vN = data.ESKF.vN;
eskf_vE = data.ESKF.vE;
eskf_vD = data.ESKF.vD;
eskf_pN = data.ESKF.pN;
eskf_pE = data.ESKF.pE;
eskf_pD = data.ESKF.pD;
eskf_aN = data.ESKF.aN;
eskf_aE = data.ESKF.aE;
eskf_aD = data.ESKF.aD;
len_eskf = length(eskf_t);

eskfData.time = eskf_t-eskf_t(1,1);
eskfData.signals.values = [eskf_roll eskf_pitch eskf_yaw eskf_vN eskf_vE eskf_vD eskf_pN eskf_pE eskf_pD eskf_aN eskf_aE eskf_aD];


%%
figure
plot(eskf_t,eskf_pN,eskf_t,eskf_pE,eskf_t,eskf_pD)

%% 参数读取
control_param_att.roll_Kp = data.PARM.Value(2);
control_param_att.roll_acc_max = data.PARM.Value(3);
control_param_att.roll_rate_Kp = data.PARM.Value(4);
control_param_att.roll_rate_Ki = data.PARM.Value(5);
control_param_att.roll_rate_Kd = data.PARM.Value(6);
control_param_att.roll_i_limit = data.PARM.Value(7);

control_param_att.pitch_Kp = data.PARM.Value(8);
control_param_att.pitch_acc_max = data.PARM.Value(9);
control_param_att.pitch_rate_Kp = data.PARM.Value(10);
control_param_att.pitch_rate_Ki = data.PARM.Value(11);
control_param_att.pitch_rate_Kd = data.PARM.Value(12);
control_param_att.pitch_i_limit = data.PARM.Value(13);

control_param_att.yaw_Kp = data.PARM.Value(14);
control_param_att.yaw_acc_max = data.PARM.Value(15);
control_param_att.yaw_rate_Kp = data.PARM.Value(16);
control_param_att.yaw_rate_Ki = data.PARM.Value(17);
control_param_att.yaw_rate_Kd = data.PARM.Value(18);
control_param_att.yaw_i_limit = data.PARM.Value(19);

control_param_att.roll_eso_b0 = data.PARM.Value(20);
control_param_att.ctrl_pwm_min = data.PARM.Value(21);
control_param_att.ctrl_pwm_max = data.PARM.Value(22);

control_param_pos.z_p = data.PARM.Value(23);
control_param_pos.z_acc_max = data.PARM.Value(24);                       %z轴允许的最大加速度;
control_param_pos.z_vel_p = data.PARM.Value(25);
control_param_pos.z_vel_i = data.PARM.Value(26);
control_param_pos.z_vel_d = data.PARM.Value(27);
control_param_pos.z_i_limit = data.PARM.Value(28);

control_param_pos.x_p = data.PARM.Value(29);
control_param_pos.x_acc_max = data.PARM.Value(30);
control_param_pos.x_vel_p = data.PARM.Value(31);
control_param_pos.x_vel_i = data.PARM.Value(32);
control_param_pos.x_vel_d = data.PARM.Value(33);
control_param_pos.x_i_limit = data.PARM.Value(34);

control_param_pos.y_p = data.PARM.Value(35);
control_param_pos.y_acc_max = data.PARM.Value(36);
control_param_pos.y_vel_p = data.PARM.Value(37);
control_param_pos.y_vel_i = data.PARM.Value(38);
control_param_pos.y_vel_d = data.PARM.Value(39);
control_param_pos.y_i_limit = data.PARM.Value(40);

control_param_pos.xy_spd_td_r = data.PARM.Value(41);


state_machine_param.idel_pwm = data.PARM.Value(42);                   %怠速，油门PWM 百分比
state_machine_param.Rac = data.PARM.Value(43);
state_machine_param.xy_rad = data.PARM.Value(44);
state_machine_param.z_rad = data.PARM.Value(45);
state_machine_param.max_acc_xy = data.PARM.Value(46);
state_machine_param.max_jerk = data.PARM.Value(47);
state_machine_param.max_xy_speed = data.PARM.Value(48);
state_machine_param.max_acc_xy_radius_scale = data.PARM.Value(49);
state_machine_param.min_velocity = data.PARM.Value(50);
state_machine_param.acceptance_angle_cos = data.PARM.Value(51);
state_machine_param.xy_err_max = data.PARM.Value(52);
state_machine_param.climb_acc_max = data.PARM.Value(53);                %最大爬升加速度
state_machine_param.climb_rate = data.PARM.Value(54);                 %最大爬升速率
state_machine_param.cruise_speed = data.PARM.Value(55);               %额定巡航速度
state_machine_param.yaw_rate_max = data.PARM.Value(56);               %最大航向角速率，单位rad/s
state_machine_param.psi_rate_td_r = data.PARM.Value(57);

state_machine_param.roll_deadzone = data.PARM.Value(58);
state_machine_param.pitch_deadzone = data.PARM.Value(59);
state_machine_param.yaw_deadzone = data.PARM.Value(60);
state_machine_param.throttle_deadzone = data.PARM.Value(61);
state_machine_param.takeoff_pwm = data.PARM.Value(62);
state_machine_param.land_threshold = data.PARM.Value(63);

state_machine_param.hover_throttle = data.PARM.Value(64);
state_machine_param.hover_x_kp = data.PARM.Value(65);
state_machine_param.hover_y_kp = data.PARM.Value(66);
state_machine_param.hover_z_kp = data.PARM.Value(67);
