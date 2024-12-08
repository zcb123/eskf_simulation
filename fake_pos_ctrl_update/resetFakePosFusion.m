function resetFakePosFusion()
    global states 
    global time_last_imu time_last_fake_pos_fuse last_known_posNE
    last_known_posNE = states.pos(1:2,1);
	resetHorizontalPositionToLastKnown();
	resetHorizontalVelocityToZero();
	time_last_fake_pos_fuse = time_last_imu;


end

