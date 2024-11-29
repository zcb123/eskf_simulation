function yaw = getEulerYaw(dcm)
    
    yaw = atan2(dcm(2,1),dcm(1,1));

end