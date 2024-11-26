

global P P_M dt_ekf_avg;
P = zeros(23,23);
P(1,1) = params.initial_tilt_err^2;
P(2,2) = P(1,1);
P(3,3) = P(1,1);
P(4,4) = params.gps_vel_noise;
P(5,5) = P(4,4);
P(6,6) = P(4,4);
P(7,7) = params.gps_pos_noise;
P(8,8) = P(7,7);
P(9,9) = data.RTK.pdop(1,1)^2 - data.RTK.hdop(1,1)^2;                %等于gps垂直精度(vacc)的平方
P(10,10) = params.switch_on_gyro_bias*dt_ekf_avg;
P(11,11) = P(10,10);
P(12,12) = P(10,10);
P(13,13) = params.switch_on_accel_bias*dt_ekf_avg;
P(14,14) = P(13,13);
P(15,15) = P(13,13);

P(16,16) = params.mag_noise^2;
P(17,17) = P(16,16);
P(18,18) = P(16,16);
P(19,19) = params.mag_noise^2;
P(20,20) = P(19,19);
P(21,21) = P(19,19);
P(22,22) = params.initial_wind_uncertainty;
P(23,23) = P(22,22);

P_M = P;
