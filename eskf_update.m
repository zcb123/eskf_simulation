function eskf_update(updated)


    
    
    if updated
        predictCovariance();
        predictCovariance_Matrix();
        predictState();

        runYawEKFGSF();
        controlGpsFusion();
        controlHeightFusion();

    end



    

    calculateOutputStates();
end

