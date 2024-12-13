function ret = hasHorizontalAidingTimedOut()
    
    global time_last_hor_pos_fuse time_last_hor_vel_fuse time_last_of_fuse;
    global params;
    
    ret = isTimedOut(time_last_hor_pos_fuse,params.reset_timeout_max)...
        && isTimedOut(time_last_hor_vel_fuse, params.reset_timeout_max)...
	    && isTimedOut(time_last_of_fuse, params.reset_timeout_max); %time_last_of_fuse光流融合必定超时,最后这里恒为true

end