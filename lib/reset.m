clear predictState predictCovariance predictCovariance_Matrix ...
    fuseVelocityWithLevelArm fuseVelocityWithLevelArm_Matrix;   

len_update = length(imu_delta_t);

init_states();

run("P_init.m");

clear yawEstimator;
global yawEstimator;
yawEstimator = EKFGSF_YAW();