function fuse(K,innovation)

global states;

dTheta = -K(1:3,1)*innovation;
tmp_q = Quaternion_from_AxisAngle_3arg(dTheta);
states.quat_nominal = quatMult(tmp_q,states.quat_nominal);
states.quat_nominal = quat_normalize(states.quat_nominal);
states.vel = states.vel - K(4:6,1)*innovation;
states.pos = states.pos - K(7:9,1)*innovation;
states.delta_ang_bias = states.delta_ang_bias - K(10:12,1)*innovation;
states.delta_vel_bias = states.delta_vel_bias - K(13:15,1)*innovation;
states.mag_I = states.mag_I - K(16:18,1)*innovation;
states.mag_B = states.mag_B - K(19:21,1)*innovation;
states.wind_vel = states.wind_vel - K(22:23,1)*innovation;


% assignin("base","states_delta_ang_bias",states.delta_ang_bias);
end