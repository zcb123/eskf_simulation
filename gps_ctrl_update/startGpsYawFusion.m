function startGpsYawFusion()

    global control_status;
    if resetYawToGps()
        control_status.flags.yaw_align = true;
        control_status.flags.mag_dec = false;
        stopEvYawFusion();
        stopMagHdgFusion();
        stopMag3DFusion();
        control_status.flags.gps_yaw = true;
    end
end

