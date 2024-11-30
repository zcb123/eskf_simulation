function ret = canResetMagHeading()


    global control_status params;
    
    ret = ~control_status.flags.mag_field_disturbed && params.mag_fusion_type ~= 5;%NONE = 5

end

