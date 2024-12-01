function setMagData(mag_data,mag_index)

    global mag_sample_delayed;
    mag_sample_delayed.time_us = mag_data.t(mag_index,1);
    mag_sample_delayed.mag = double([mag_data.X(mag_index,1) mag_data.Y(mag_index,1) mag_data.Z(mag_index,1)]')/0.003;
    
end


