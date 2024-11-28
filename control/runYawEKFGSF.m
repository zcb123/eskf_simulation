function runYawEKFGSF(imu_sample_updated,gps_data,data_ready,gps_index)

    global yawEstimator;
    TAS = 15;
    yawEstimator.Update(imu_sample_updated,TAS);

    if data_ready 
    
        yawEstimator.setVelocity([gps_data.vN(gps_index,1) gps_data.vE(gps_index,1)]',0.01);     %本来这里应该设置vacc的速度精度 代码中给了定值0.01

    end
end