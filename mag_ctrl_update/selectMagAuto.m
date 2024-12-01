function selectMagAuto()


    check3DMagFusionSuitability();  %检测磁场融合的稳定性
    
    if canUse3DMagFusion()

        startMag3DFusion();
        disp("start Mag 3D fusion")
    else
        startMagHdgFusion();
        disp("start Mag Hdg fusion")
    end


end

