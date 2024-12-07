function ret = isStrongMagneticDisturbance()
    global control_status;
    
    ret = control_status.flags.mag_field_disturbed;


end

