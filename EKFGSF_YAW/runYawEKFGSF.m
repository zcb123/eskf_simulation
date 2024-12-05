function runYawEKFGSF()
    global FLT_EPSILON;
    global yawEstimator control_status;
    global imu_sample_delayed gps_sample_delayed;
    global gps_data_ready;
    TAS = 0;

    yawEstimator.Update(imu_sample_delayed,control_status.flags.in_air,TAS);    

    if gps_data_ready && (gps_sample_delayed.vacc > FLT_EPSILON) 
        if ~isnan(gps_sample_delayed.vel(1))&&~isnan(gps_sample_delayed.vel(2))
       
            yawEstimator.setVelocity(gps_sample_delayed.vel(1:2,1),gps_sample_delayed.vacc);     %本来这里应该设置vacc的速度精度 代码中给了定值0.01
        
        end
    end
end