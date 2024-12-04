function applyCorrectionToOutputBuffer()

    global output_buffer output_new;

    for i = 1:output_buffer.len
        output_buffer(i,1).vel = output_buffer(i,1).vel + vel_correction;
        output_buffer(i,1).pos = output_buffer(i,1).pos + pos_correction;
    end

    output_new = output_buffer.get_newset();
        

end

