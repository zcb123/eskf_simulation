function alignOutputFilter()

    global states output_new output_buffer;
    
    output_delayed = output_buffer.get_oldest();

    q_delta =  QuatMult(states.quat_nominal,Quaternion_Inverse(output_delayed.quat_nominal));
    q_delta = quat_normalize(q_delta);

    vel_delta = states.vel - output_delayed.vel;
    pos_delta = states.pos - output_delayed.pos;

    for i = 1:output_buffer.len 
        output_buffer.elements{i,1}.quat_nominal = QuatMult(q_delta , output_buffer.elements{i,1}.quat_nominal);
		output_buffer.elements{i,1}.quat_nominal = quat_normalize(output_buffer.elements{i,1}.quat_nominal);
		output_buffer.elements{i,1}.vel = output_buffer.elements{i,1}.vel + vel_delta;
		output_buffer.elements{i,1}.pos = output_buffer.elements{i,1}.pos + pos_delta;
    end

    output_new = output_buffer.get_newest();
    
end

