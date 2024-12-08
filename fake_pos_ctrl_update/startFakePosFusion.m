function startFakePosFusion()

    global using_synthetic_position fuse_hpos_as_odom
    if (~using_synthetic_position) 
		using_synthetic_position = true;
		fuse_hpos_as_odom = false; % TODO: needed?
		resetFakePosFusion();
    end

end
