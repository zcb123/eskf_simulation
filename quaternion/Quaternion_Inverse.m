function quat_inverse = Quaternion_Inverse(quat)
    quat_start = [quat(1) -quat(2) -quat(3) -quat(4)]';
    quat_len = sqrt(quat(1)^2 + quat(2)^2 + quat(3)^2 + quat(4)^2);
    quat_inverse = quat_start/quat_len;
end