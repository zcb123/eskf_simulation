classdef EKFGSF_YAW < handle
    properties
        % Parameters - these could be made tuneable
	    gyro_noise = 1.0e-1; 	% yaw rate noise used for covariance prediction (rad/sec)
	    accel_noise = 2.0;		% horizontal accel noise used for covariance prediction (m/sec**2)
	    tilt_gain = 0.2;		% gain from tilt error to gyro correction for complementary filter (1/sec)  陀螺仪互补校正的系数
	    gyro_bias_gain = 0.04;	% gain applied to integral of gyro correction for complementary filter (1/sec)
    
	    % Declarations used by the bank of N_MODELS_EKFGSF AHRS complementary filters
    
	    delta_ang;	% IMU delta angle (rad)
	    delta_vel;	% IMU delta velocity (m/s)
	    delta_ang_dt;	% _delta_ang integration time interval (sec)
	    delta_vel_dt;	% _delta_vel integration time interval (sec)
        true_airspeed;	% true airspeed used for centripetal accel compensation (m/s)
        
        ahrs_ekf_gsf_1 ;

        ahrs_ekf_gsf;
	      
	    ahrs_ekf_gsf_tilt_aligned;	% true the initial tilt alignment has been calculated
	    ahrs_accel_fusion_gain;	% gain from accel vector tilt error to rate gyro correction used by AHRS calculation
	    ahrs_accel;			% low pass filtered body frame specific force vector used by AHRS calculation (m/s/s)
	    ahrs_accel_norm;		% length of _ahrs_accel specific force vector used by AHRS calculation (m/s/s)
    
	    % Declarations used by a bank of N_MODELS_EKFGSF EKFs
        ekf_gsf_1;
	    ekf_gsf ;
    
	    vel_data_updated;	% true when velocity data has been updated
	    run_ekf_gsf;		% true when operating condition is suitable for to run the GSF and EKF models and fuse velocity data
	    vel_NE;        % NE velocity observations (m/s)
	    vel_accuracy;     % 1-sigma accuracy of velocity observations (m/s)
	    ekf_gsf_vel_fuse_started; % true when the EKF's have started fusing velocity data and the prediction and update processing is active
    

	    % Declarations used by the Gaussian Sum Filter (GSF) that combines the individual EKF yaw estimates
    
	    model_weights;
	    gsf_yaw; 		% yaw estimate (rad)
	    gsf_yaw_variance; 	% variance of yaw estimate (rad^2)
       
	    % return the probability of the state estimate for the specified EKF assuming a gaussian error distribution
    end
    methods(Access=public)

        function obj = EKFGSF_YAW()

            obj.delta_ang = [ 0 0 0]';
            obj.delta_vel = [ 0 0 0]';
            obj.model_weights = zeros(5,1);
            obj.ahrs_ekf_gsf_tilt_aligned = logical(false);
            obj.gsf_yaw = 0;
            obj.gsf_yaw_variance = 0;
            obj.ahrs_accel= [0 0 0]';
            obj.tilt_gain = 0.2;
            obj.vel_data_updated = logical(false);
            obj.ekf_gsf_vel_fuse_started = logical(false);
            obj.ahrs_accel_fusion_gain = single(0);
            obj.vel_NE = zeros(2,1);
            obj.vel_accuracy = 0;
            obj.true_airspeed = 0;
            obj.ahrs_accel_norm = 0;

            obj.ahrs_ekf_gsf_1 = struct('R',zeros(3,3),...
                                'gyro_bias',zeros(3,1),...
                                'aligned',logical(true),...
                                'vel_NE',zeros(1,2),...
                                'fuse_gps',logical(true),...
                                'accel_dt',0);
            obj.ahrs_ekf_gsf = [obj.ahrs_ekf_gsf_1;obj.ahrs_ekf_gsf_1;obj.ahrs_ekf_gsf_1;obj.ahrs_ekf_gsf_1;obj.ahrs_ekf_gsf_1];
           
            
            obj.ekf_gsf_1 = struct('X',zeros(3,1),...
                            'P',zeros(3,3),...
                            'S_inverse',zeros(2,2),...
                            'S_det_inverse',0,...
                            'innov',zeros(2,1));
            obj.ekf_gsf = [obj.ekf_gsf_1;obj.ekf_gsf_1;obj.ekf_gsf_1;obj.ekf_gsf_1;obj.ekf_gsf_1];

        end

        obj = Update(obj,imu_sample,airspeed) 
        
        function obj = setVelocity(obj,vel,accuracy)
            obj.vel_NE = vel;
            obj.vel_accuracy = accuracy;
            obj.vel_data_updated = true;
        end

    end

   methods(Access = private)
      
        obj = initialiseEKFGSF(obj);

        % update specified AHRS rotation matrix using IMU and optionally true airspeed data
	    obj = ahrsPredict(obj,model_index);
    
	    % align all AHRS roll and pitch orientations using IMU delta velocity vector
	    obj = ahrsAlignTilt(obj);
    
	    % align all AHRS yaw orientations to initial values
	    obj = ahrsAlignYaw(obj);
            
	    % Efficient propagation of a delta angle in body frame applied to the body to earth frame rotation matrix
	    obj = ahrsPredictRotMat(obj,Rot,g);
        
        obj = ahrsCalcAccelGain(obj);

        obj = predictEKF(obj,model_index);
	    % update state and covariance for the specified EKF using a NE velocity measurement
	    % return false if update failed
	    [obj,res] = updateEKF(obj,model_index);
        
        res = gaussianDensity(obj,model_index);

   end
end