function rot_out = updateYawInRotMat(yaw,rot_in)

    euler321 = Dcm2Euler(rot_in);

    euler321(3) = yaw;

    rot_out = Euler2Dcm(euler321);


end