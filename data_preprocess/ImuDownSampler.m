classdef ImuDownSampler<handle
    properties
    
         imu_down_sampled
         delta_angle_accumulated
        
         accumulated_samples
         required_samples
        
         target_dt_us
        
         target_dt_s
         min_dt_s
        
         delta_ang_dt_avg

    end
    
    methods
        %构造函数
        function obj = ImuDownSampler(target_dt_us)
            obj.target_dt_us = target_dt_us;    
            obj.delta_ang_dt_avg = 0.0025;
            reset(obj);
        end

        function ret = update(obj,imu_sample_new)
            obj.delta_ang_dt_avg = 0.9*obj.delta_ang_dt_avg + 0.1*imu_sample_new.delta_ang_dt;
            % accumulate time deltas
	        obj.imu_down_sampled.time_us = imu_sample_new.time_us;
	        obj.imu_down_sampled.delta_ang_dt = obj.imu_down_sampled.delta_ang_dt + imu_sample_new.delta_ang_dt;
	        obj.imu_down_sampled.delta_vel_dt = obj.imu_down_sampled.delta_vel_dt + imu_sample_new.delta_vel_dt;
            % 这里怎么只有vel_cliping，没有ang_clipping
        %     obj.imu_down_sampled.delta_ang_clipping(1) = obj.imu_down_sampled.delta_ang_clipping(1) | imu_sample_new.delta_ang_clipping(1);
        %     obj.imu_down_sampled.delta_ang_clipping(2) = obj.imu_down_sampled.delta_ang_clipping(2) | imu_sample_new.delta_ang_clipping(2);
        %     obj.imu_down_sampled.delta_ang_clipping(3) = obj.imu_down_sampled.delta_ang_clipping(3) | imu_sample_new.delta_ang_clipping(3);
	        obj.imu_down_sampled.delta_vel_clipping(1) = obj.imu_down_sampled.delta_vel_clipping(1) | imu_sample_new.delta_vel_clipping(1);
	        obj.imu_down_sampled.delta_vel_clipping(2) = obj.imu_down_sampled.delta_vel_clipping(2) | imu_sample_new.delta_vel_clipping(2);
	        obj.imu_down_sampled.delta_vel_clipping(3) = obj.imu_down_sampled.delta_vel_clipping(3) | imu_sample_new.delta_vel_clipping(3);
        
            
	        delta_q = Quaternion_from_AxisAngle_3arg(imu_sample_new.delta_ang);     %delta_ang通过四元数积累误差      
	        obj.delta_angle_accumulated = quatMult(obj.delta_angle_accumulated,delta_q);
            obj.delta_angle_accumulated = quat_normalize(obj.delta_angle_accumulated);
        
	        % rotate the accumulated delta velocity data forward each time so it is always in the updated rotation frame
	        % 将速度增量向前旋转，使其能始终处于更新的机体坐标系中
	        delta_R = Quat2Tbn(quat_inverse(delta_q));
	        obj.imu_down_sampled.delta_vel = delta_R*obj.imu_down_sampled.delta_vel;
        
	        % accumulate the most recent delta velocity data at the updated rotation frame
	        % 在更新的旋转坐标系中积分最新的速度增量数据
	        % assume effective sample time is halfway between the previous and current rotation frame
	        % 假设有效采样时间位于前一个旋转帧和当前旋转帧的中间
	        obj.imu_down_sampled.delta_vel = obj.imu_down_sampled.delta_vel + (imu_sample_new.delta_vel + delta_R*imu_sample_new.delta_vel)*0.5;
            
	        obj.accumulated_samples = obj.accumulated_samples + 1;
        
            ret = logical(false);
	        % required number of samples accumulated and the total time is at least half of the target
	        %  OR total time already exceeds the target
	        if ((obj.accumulated_samples >= obj.required_samples && obj.imu_down_sampled.delta_ang_dt > obj.min_dt_s)...
	            || (obj.imu_down_sampled.delta_ang_dt > obj.target_dt_s)) 
        
		        obj.imu_down_sampled.delta_ang = Quaternion_to_AxisAngle(obj.delta_angle_accumulated,single(1e-7))';
		        ret = logical(true);
            end       
        end
        function res = getDownSampledImuAndTriggerReset(obj)
            res = obj.imu_down_sampled;
            reset(obj);
        end

        function obj = reset(obj)

            obj.imu_down_sampled = struct('time_us',uint64(0),...
                            'delta_ang',single([0 0 0]'),...
                            'delta_vel',single([0 0 0]'),...)
                            'delta_ang_dt',single(0),...
                            'delta_vel_dt',single(0),...
                            'delta_ang_clipping',logical([0 0 0]'),...
                            'delta_vel_clipping',logical([0 0 0]'));

            obj.delta_angle_accumulated = single([1 0 0 0]');
            obj.accumulated_samples = 0;

%             obj.delta_ang_dt_avg = dt_imu_avg;
            obj.target_dt_s = saturation(obj.target_dt_us,1000, 100000) * 1e-6;		%0703:4000 0831:8000

            obj.required_samples = max(round(obj.target_dt_s / obj.delta_ang_dt_avg), 1);					%0.004

            obj.target_dt_s = obj.required_samples * obj.delta_ang_dt_avg;
            obj.min_dt_s = max(obj.delta_ang_dt_avg * (obj.required_samples - 1), obj.delta_ang_dt_avg * 0.5);
        
        end

    end
end



