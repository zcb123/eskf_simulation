function stopMag3DFusion()
    global control_status;
    
    if (control_status.flags.mag_3D) 
		saveMagCovData();
		control_status.flags.mag_3D = false;
    end
end

