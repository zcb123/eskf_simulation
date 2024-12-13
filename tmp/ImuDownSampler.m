function updated = ImuDownSampler(imu_sample_new,required_samples,target_dt_s,min_dt_s)
    persistent delta_ang_dt_avg;
    if isempty(delta_ang_dt_avg)
        delta_ang_dt_avg = single([ 0 0 0]');
    end
    % 这个值接近于imu_sample_new.delta_ang_dt
	delta_ang_dt_avg = 0.9 * delta_ang_dt_avg + 0.1 * imu_sample_new.delta_ang_dt;	%截止频率fc = 0.1*fs / 2*pi	fs=250时,fc = 3.9789Hz	

	% accumulate time deltas
	imu_down_sampled.time_us = imu_sample_new.time_us;
	imu_down_sampled.delta_ang_dt = imu_down_sampled.delta_ang_dt + imu_sample_new.delta_ang_dt;
	imu_down_sampled.delta_vel_dt = imu_down_sampled.delta_vel_dt + imu_sample_new.delta_vel_dt;
	imu_down_sampled.delta_vel_clipping(1) = imu_down_sampled.delta_vel_clipping(1) | imu_sample_new.delta_vel_clipping(1);
	imu_down_sampled.delta_vel_clipping(2) = imu_down_sampled.delta_vel_clipping(2) | imu_sample_new.delta_vel_clipping(2);
	imu_down_sampled.delta_vel_clipping(3) = imu_down_sampled.delta_vel_clipping(3) | imu_sample_new.delta_vel_clipping(3);

	% use a quaternion to accumulate delta angle data
	% 用四元数积累角度误差
	% this quaternion represents the rotation from the start to end of the accumulation period
	% 这个四元数表示从累积的开始到结束的旋转
    persistent delta_angle_accumulated
    if isempty(delta_angle_accumulated)
        delta_angle_accumulated = single([1 0 0 0]');   %这是一个四元数
    end
	delta_q = Quaternion_from_AxisAngle_3arg(imu.delta_ang);
	delta_angle_accumulated = quatMult(delta_angle_accumulated,delta_q);
    delta_angle_accumulated = quat_normalize(delta_angle_accumulated);

	% rotate the accumulated delta velocity data forward each time so it is always in the updated rotation frame
	% 将速度增量向前旋转，使其能始终处于更新的机体坐标系中
	delta_R = Quat2Tbn(Quaternion_Inverse(delta_q));
	imu_down_sampled.delta_vel = delta_R * imu_down_sampled.delta_vel;

	% accumulate the most recent delta velocity data at the updated rotation frame
	% 在更新的旋转坐标系中积分最新的速度增量数据
	% assume effective sample time is halfway between the previous and current rotation frame
	% 假设有效采样时间位于前一个旋转帧和当前旋转帧的中间
	imu_down_sampled.delta_vel = imu_down_sampled.delta_vel + (imu.delta_vel + delta_R * imu.delta_vel) * 0.5;
    persistent accumulated_samples
    if isempty(accumulated_samples)
        accumulated_samples = single(0);
    end
	accumulated_samples = accumulated_samples + 1;

    updated = logical(false);
	% required number of samples accumulated and the total time is at least half of the target
	%  OR total time already exceeds the target
	if ((accumulated_samples >= required_samples && imu_down_sampled.delta_ang_dt > min_dt_s)...
	    || (imu_down_sampled.delta_ang_dt > target_dt_s)) 

		imu_down_sampled.delta_ang = Quaternion_to_AxisAngle(delta_angle_accumulated,1e-7);
		updated = logical(true);
    end


end