function startMag3DFusion()
    
    global P control_status;
    global saved_mag_bf_variance saved_mag_ef_ne_covmat saved_mag_ef_d_variance;
    if ~control_status.flags.mag_3D
        control_status.flags.mag_hdg = false;
%         P(16,16) = 0;
%         P(17,17) = 0;
%         P(18,18) = 0;
%         P(19,19) = 0;
%         P(20,20) = 0;
%         P(21,21) = 0;

        P(19,19) = saved_mag_bf_variance(1);
        P(20,20) = saved_mag_bf_variance(2);
        P(21,21) = saved_mag_bf_variance(3);
        P(16:17,16:17) = saved_mag_ef_ne_covmat;
        P(18,18) = saved_mag_ef_d_variance;

        control_status.flags.mag_3D = true;
    end


end