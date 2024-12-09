function reset_eskf()
    global states output_new delta_angle_corr control_status control_status_prev ang_rate_delayed_raw prev_dvel_bias_var;

    states.quat_nominal = [1 0 0 0]';
    states.vel = zeros(3,1);
    states.pos = zeros(3,1);
    states.delta_ang_bias = zeros(3,1);
    states.delta_vel_bias = zeros(3,1);
    states.mag_I = zeros(3,1);
    states.mag_B = zeros(3,1);
    states.wind_vel = zeros(2,1);

    output_new.quat_nominal = [1 0 0 0]';
    output_new.vel = zeros(3,1);
    output_new.pos = zeros(3,1);

    delta_angle_corr = zeros(3,1);

%   等用到了再说
%     	_range_sensor.setPitchOffset(_params.rng_sens_pitch);
% 	_range_sensor.setCosMaxTilt(_params.range_cos_max_tilt);
% 	_range_sensor.setQualityHysteresis(_params.range_valid_quality_s);
    
    control_status.flags = zerosStruct(control_status.flags);
    control_status_prev.flags = zerosStruct(control_status_prev.flags);

    control_status.flags.in_air = true;
    control_status_prev.flags.in_air = true;

    ang_rate_delayed_raw = zeros(3,1);
    
    prev_dvel_bias_var = zeros(3,1);

    resetGpsDriftCheckFilters();

end