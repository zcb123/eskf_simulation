function checkMagDeclRequired()

    global params control_status;
    global FUSE_DECL;
    user_selected = logical(bitand(params.mag_declination_source , FUSE_DECL));
	not_using_ne_aiding = ~control_status.flags.gps;
    
	control_status.flags.mag_dec = (control_status.flags.mag_3D && (not_using_ne_aiding || user_selected));


end