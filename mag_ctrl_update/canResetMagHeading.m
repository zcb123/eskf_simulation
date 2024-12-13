function ret = canResetMagHeading()


    global control_status params NONE;
    
    ret = ~control_status.flags.mag_field_disturbed && params.mag_fusion_type ~= NONE;%NONE = 5

end

