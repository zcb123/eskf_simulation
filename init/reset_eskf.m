
function reset_eskf()

    global states output_new ;

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

    
    % 后面还有一点其他的初始化 暂时不写

end

