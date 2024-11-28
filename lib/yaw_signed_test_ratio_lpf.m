function res = yaw_signed_test_ratio_lpf(val)

    persistent last_val
    if isempty(last_val)
        last_val = 0;
    end

    res = (1-0.1)*last_val + 0.1*val;
    
    last_val = val;
end