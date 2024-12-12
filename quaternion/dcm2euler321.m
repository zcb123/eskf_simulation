function euler = dcm2euler321(dcm)
    phy = atan2(dcm(3,2),dcm(3,3));
    theta = -asin(dcm(3,1));
    yaw = atan2(dcm(2,1),dcm(1,1));

    euler = [phy theta yaw]';

end