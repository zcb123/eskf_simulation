function updated = setIMUData(imu_sample_new)

global imu_sample_delayed dt_imu_avg time_last_imu;
global required_samples target_dt_s min_dt_s;


dt = double(imu_sample_new.time_us - time_last_imu)/1e6;
dt = saturation(dt,1e-4,0.02);

time_last_imu = imu_sample_new.time_us;

if(time_last_imu>0)
    dt_imu_avg = 0.8*dt_imu_avg+0.2*dt;
end

%%  下采样
    persistent imu_down_sampled;
    if isempty(imu_down_sampled)
        imu_down_sampled = struct('time_us',uint64(0),...
                            'delta_ang',single([0 0 0]'),...
                            'delta_vel',single([0 0 0]'),...)
                            'delta_ang_dt',single(0),...
                            'delta_vel_dt',single(0),...
                            'delta_ang_clipping',logical([0 0 0]'),...
                            'delta_vel_clipping',logical([0 0 0]'));
    end
    persistent delta_ang_dt_avg_last;
    if isempty(delta_ang_dt_avg_last)
        delta_ang_dt_avg_last = 0.0025;
    end
    delta_ang_dt_avg = 0.9*delta_ang_dt_avg_last + 0.1*imu_sample_new.delta_ang_dt;
    delta_ang_dt_avg_last = delta_ang_dt_avg;

	% accumulate time deltas
	imu_down_sampled.time_us = imu_sample_new.time_us;
	imu_down_sampled.delta_ang_dt = imu_down_sampled.delta_ang_dt + imu_sample_new.delta_ang_dt;
	imu_down_sampled.delta_vel_dt = imu_down_sampled.delta_vel_dt + imu_sample_new.delta_vel_dt;
    % 这里怎么只有vel_cliping，没有ang_clipping
%     imu_down_sampled.delta_ang_clipping(1) = imu_down_sampled.delta_ang_clipping(1) | imu_sample_new.delta_ang_clipping(1);
%     imu_down_sampled.delta_ang_clipping(2) = imu_down_sampled.delta_ang_clipping(2) | imu_sample_new.delta_ang_clipping(2);
%     imu_down_sampled.delta_ang_clipping(3) = imu_down_sampled.delta_ang_clipping(3) | imu_sample_new.delta_ang_clipping(3);
	imu_down_sampled.delta_vel_clipping(1) = imu_down_sampled.delta_vel_clipping(1) | imu_sample_new.delta_vel_clipping(1);
	imu_down_sampled.delta_vel_clipping(2) = imu_down_sampled.delta_vel_clipping(2) | imu_sample_new.delta_vel_clipping(2);
	imu_down_sampled.delta_vel_clipping(3) = imu_down_sampled.delta_vel_clipping(3) | imu_sample_new.delta_vel_clipping(3);

    persistent delta_angle_accumulated
    if isempty(delta_angle_accumulated)
        delta_angle_accumulated = single([1 0 0 0]');   %这是一个四元数
    end
	delta_q = Quaternion_from_AxisAngle_3arg(imu_sample_new.delta_ang);     %delta_ang通过四元数积累误差      
	delta_angle_accumulated = quatMult(delta_angle_accumulated,delta_q);
    delta_angle_accumulated = quat_normalize(delta_angle_accumulated);

	% rotate the accumulated delta velocity data forward each time so it is always in the updated rotation frame
	% 将速度增量向前旋转，使其能始终处于更新的机体坐标系中
	delta_R = Quat2Tbn(quat_inverse(delta_q));
	imu_down_sampled.delta_vel = delta_R*imu_down_sampled.delta_vel;

	% accumulate the most recent delta velocity data at the updated rotation frame
	% 在更新的旋转坐标系中积分最新的速度增量数据
	% assume effective sample time is halfway between the previous and current rotation frame
	% 假设有效采样时间位于前一个旋转帧和当前旋转帧的中间
	imu_down_sampled.delta_vel = imu_down_sampled.delta_vel + (imu_sample_new.delta_vel + delta_R*imu_sample_new.delta_vel)*0.5;
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

		imu_down_sampled.delta_ang = Quaternion_to_AxisAngle(delta_angle_accumulated,single(1e-7))';
		updated = logical(true);
    end
    ang_out = imu_down_sampled.delta_ang';
    vel_out = imu_down_sampled.delta_vel';
    quat_angle_out = delta_angle_accumulated';
    assignin("base","ang_out",ang_out);
    assignin("base","vel_out",vel_out);
    assignin("base","quat_angle_out",quat_angle_out);
%% 
if updated

    imu_sample_delayed = imu_down_sampled; %这里把环形缓冲区简化了，假设缓冲区的大小为1


    accumulated_samples = single(0);

    delta_angle_accumulated = single([1 0 0 0]');
    imu_down_sampled.delta_ang = single([0 0 0]');
    imu_down_sampled.delta_vel = single([0 0 0]');
    imu_down_sampled.delta_ang_dt = single(0);
    imu_down_sampled.delta_vel_dt = single(0);
    imu_down_sampled.delta_vel_clipping = logical([ 0 0 0]');
	
    reset_setIMUData(delta_ang_dt_avg);
end

    
	


