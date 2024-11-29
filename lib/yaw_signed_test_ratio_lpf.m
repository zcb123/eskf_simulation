function res = yaw_signed_test_ratio_lpf(val)
    global yaw_signed_test_ratio_alpha;
    persistent last_val
    if isempty(last_val)
        last_val = 0;
    end

    res = (1-yaw_signed_test_ratio_alpha)*last_val + yaw_signed_test_ratio_alpha*val;
    
    last_val = val;
end