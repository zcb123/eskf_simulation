function alignOutputFilter()

    global states output_new output_buffer;
    output_delayed = output_buffer.get_oldest();

    q_delta =  QuatMult(states.quat_nominal,Quaternion_Inverse(output_delayed.quat_nominal));
    q_delta = quat_normalize(q_delta);

    vel_delta = states.vel - output_delayed.vel;
    pos_delta = states.pos - output_delayed.pos;

    for i = 1:output_buffer.len 
        output_buffer(i,:).quat_nominal = QuatMult(q_delta , output_buffer(i,:).quat_nominal);
		output_buffer(i,:).quat_nominal = quat_normalize(output_buffer(i,:).quat_nominal);
		output_buffer(i,:).vel = output_buffer(i,:).vel + vel_delta;
		output_buffer(i,:).pos = output_buffer(i,:).pos + pos_delta;
    end

    output_new = output_buffer.get_newest();
    
end

