function controlFusionModes()
    global control_status;
    if~control_status.flags.tilt_align
        angle_err_var_vec = calcRotVecVariances();
        if ((angle_err_var_vec(0) + angle_err_var_vec(1)) < sq(radians(3.0))) 
            control_status.flags.tilt_align = true;
        end
    
    end



end

