function startMagHdgFusion()

    global control_status;
    stopMag3DFusion();
    control_status.flags.mag_hdg = true;
end


