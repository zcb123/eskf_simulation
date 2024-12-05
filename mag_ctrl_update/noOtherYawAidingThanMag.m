function ret = noOtherYawAidingThanMag()

    global control_status;

    ret = ~control_status.flags.ev_yaw && ~control_status.flags.gps_yaw;

end

