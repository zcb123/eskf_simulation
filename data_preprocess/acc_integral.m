function [int_acc,int_dt,int_flag] = acc_integral(val,dt)
    DT_MIN = 0.001;
    DT_MAX = 0.65535;
    reset_samples_min = 8;
    reset_interval_min = 0.00375;
    persistent integral_dt;
    if isempty(integral_dt)
        integral_dt = 0;
    end
    persistent integrated_samples;
    if isempty(integrated_samples)
        integrated_samples = uint8(0);
    end
    persistent last_val;
    if isempty(last_val)
        last_val = 0;
    end
    persistent alpha;
    if isempty(alpha)
        alpha = 0;
    end
    persistent last_int_dt
    if isempty(last_int_dt)
        last_int_dt = 0.004;
    end
    persistent last_int_acc
    if isempty(last_int_acc)
        last_int_acc = [0 0 0];
    end
    if dt>DT_MIN && dt+integral_dt<DT_MAX
        integrated_samples = integrated_samples + 1;
        integral_dt = integral_dt + dt;
        alpha = alpha + (val+last_val)*dt*0.5;
%        alpha = alpha + (val+last_val)*0.5;
        last_val = val;
    else
        %reset
        alpha = 0;
        integrated_samples = 0;
        integral_dt = 0; 

        last_val = val;

    end

    int_dt = last_int_dt;
    int_flag = logical(false);
    int_acc = last_int_acc;
    if integrated_samples>reset_samples_min || integral_dt > reset_interval_min
        int_acc = alpha;
        last_int_acc = int_acc;
        int_dt = round(integral_dt*1e6); %转化成为微秒;
        last_int_dt = int_dt;
        %reset
        alpha = 0;
        integrated_samples = 0;
        integral_dt = 0;   
        int_flag = logical(true);  
    end
    
end



