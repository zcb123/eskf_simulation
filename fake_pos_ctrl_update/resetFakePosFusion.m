function resetFakePosFusion()
    global states 
    global time_last_imu time_last_fake_pos_fuse last_known_posNE
    last_known_posNE = states.pos(1:2,1);
	resetHorizontalPositionToLastKnown();%将output_buffer水平位置设置成last_known_posNE，XY位置方差置为100
	resetHorizontalVelocityToZero();    %将水平速度置为0，方差置为25
	time_last_fake_pos_fuse = time_last_imu;


end

