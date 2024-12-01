function selectMagAuto()


    check3DMagFusionSuitability();
    
    if canUse3DMagFusion()

        startMag3DFusion();
    else
        startMagHdgFusion();
    
    end


end

