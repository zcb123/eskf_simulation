%只对单位四元数成立
function quat_out = quat_inverse(quat_in)
    quat_out = [quat_in(1) -quat_in(2) -quat_in(3) -quat_in(4)];
end