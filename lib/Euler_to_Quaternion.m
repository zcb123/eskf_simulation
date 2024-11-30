%这里输入的角度是弧度
function quat = Euler_to_Quaternion(euler)
    
    quat = single([1 0 0 0]);
    % 这里输入的角度是弧度
    roll = euler(1);                    %绕x轴
    pitch = euler(2);                   %绕y轴
    yaw = euler(3);                     %绕z轴
    cr2 = cos(0.5*roll);
    cp2 = cos(0.5*pitch);
    cy2 = cos(0.5*yaw);
    sr2 = sin(0.5*roll);
    sp2 = sin(0.5*pitch);
    sy2 = sin(0.5*yaw);

    %Z-Y-X顺序旋转
    quat(1) = cy2*cp2*cr2 + sy2*sp2*sr2;
    quat(2) = cy2*cp2*sr2 - sy2*sp2*cr2;
    quat(3) = cy2*sp2*cr2 + sy2*cp2*sr2;
    quat(4) = sy2*cp2*cr2 - cy2*sp2*sr2;

end