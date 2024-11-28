function init_states()
    global states
    states.quat_nominal = [1 0 0 0]';
    states.vel = [0 0 0]';
    states.pos = [0 0 0]';
    states.delta_ang_bias = [0 0 0]';
    states.delta_vel_bias = [0 0 0]';
    states.mag_B = [0 0 0]';
    states.mag_I = [0 0 0]';
    states.wind_vel = [0 0]';
end