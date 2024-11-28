% 将角度转化到正负pi
function res = wrap_pn_pi(val)
    
    low = -pi;
    high = pi;

    if low <= val && val < high 
        res = val;
        return ;
    end

    range = high-low;
    wrap_nums = floor((val-low)/range);       %往负无穷方向取整

    res = val - range*wrap_nums;
   
end