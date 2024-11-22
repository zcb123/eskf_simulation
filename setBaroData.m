function setBaroData(baro_data)
baro_sampler = struct('time_us',uint64(0),...
                    'hgt',single(0));

persistent baro_buffer;
if isempty(baro_buffer)
        baro_buffer = [baro_sampler;
                       baro_sampler;
                       baro_sampler;];
end

persistent time_baro_last
if isempty(time_baro_last)
    time_baro_last = single(0);
end

persistent head_index;
if isempty(head_index)
    head_index = uint8(1);
end

persistent tail_index;
if isempty(tail_index)
    tail_index = uint8(1);
end

if baro_data.t - time_baro_last > min_obs_interval_us
    baro_sampler.time_us = baro_data.t;
    baro_sampler.hgt = baro_data.Hight;

    head_index = mod(head_index,3);
    head_index = head_index + 1;

    baro_buffer(head_index,:) = baro_sampler;


    
end

end

