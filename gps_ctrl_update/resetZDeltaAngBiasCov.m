function resetZDeltaAngBiasCov()

    global params dt_ekf_avg;
    
    init_delta_ang_bias_var = sq(params.switch_on_gyro_bias * dt_ekf_avg);
    uncorrelateCovarianceSetVariance(1,10,init_delta_ang_bias_var);
    
end