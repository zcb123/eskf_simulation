function resetQuatStateYaw(yaw,yaw_variance,update_buffer)
    
    global output_new output_buffer;
    global states P R_to_earth FLT_EPSILON;    	
	quat_before_reset = states.quat_nominal;

	R_to_earth = updateYawInRotMat(yaw, Quat2Tbn(states.quat_nominal));

	% calculate the amount that the quaternion has changed by
	quat_after_reset = dcm2quat(R_to_earth);
    quat_before_reset_inver_norm = quat_normalize(Quaternion_Inverse(quat_before_reset));

	q_error=(QuatMult(quat_after_reset,quat_before_reset_inver_norm));

	% update quaternion states
	states.quat_nominal = quat_after_reset;

	%uncorrelateQuatFromOtherStates();
    P(4:end,1:3) = 0;
    P(1:3,4:end) = 0;


	% record the state change
	states_reset_status.quat_change = q_error;

	% update the yaw angle variance
	if (yaw_variance > FLT_EPSILON) 
		increaseQuatYawErrVariance(yaw_variance);
%         yaw_variance = min(yaw_variance,1e-2);
%         P(3,3) = P(3,3) + yaw_variance;
	end

	% add the reset amount to the output observer buffered data
	if (update_buffer) 
		for i = 1:3 	%把缓冲区中每一个元素都更新
			output_buffer(i).quat_nominal = QuatMult(states_reset_status.quat_change,output_buffer(i).quat_nominal);
		end

		% apply the change in attitude quaternion to our newest quaternion estimate
		% which was already taken out from the output buffer
		output_new.quat_nominal =  QuatMult(states_reset_status.quat_change,output_new.quat_nominal);

	end

	% capture the reset event
	% states_reset_status.quat_counter++;


end

