function [gyro_int,int_dt,int_flag]=gyro_coningIntegral(val,dt)
    
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
    persistent alpha_first
    if isempty(alpha_first)
        alpha_first = 0;
    end
    persistent last_int_dt
    if isempty(last_int_dt)
        last_int_dt = 0.004;
    end

    
    persistent last_alpha;
    if isempty(last_alpha)
        last_alpha = 0;
    end
    persistent last_delta_alpha;
    if isempty(last_delta_alpha)
        last_delta_alpha = 0;
    end 
    persistent beta;
    if isempty(beta)
        beta = 0;
    end
    persistent alpha;
    if isempty(alpha)
        alpha = 0;
    end
    persistent last_gyro_int
    if isempty(last_gyro_int)
        last_gyro_int = 0;
    end
    %put
    if dt > DT_MIN && dt+integral_dt<DT_MAX
        
        %先梯形积分
        integrated_samples = integrated_samples + 1;
        integral_dt = integral_dt + dt;
        alpha_first = alpha_first + (val+last_val)*dt*0.5;      %alpha_first用于梯形积分的alpha
        last_val = val;
    
        %锥运动补偿
        delta_alpha = alpha_first;

        tmp = (last_alpha+last_delta_alpha/6);
        beta = beta + cross(tmp,delta_alpha)*0.5;
    
        last_delta_alpha = delta_alpha;
        last_alpha = alpha;
    
        alpha = alpha + delta_alpha;
 
    else

        alpha_first = 0;
        integrated_samples = 0;
        integral_dt = 0; 
        beta = 0;
        last_alpha = 0;

        last_val = val;
    end

    
    %integral reset 
    int_flag = logical(false);
    int_dt = last_int_dt;
    gyro_int = last_gyro_int;

    if integrated_samples>reset_samples_min || integral_dt > reset_interval_min

        gyro_int = alpha;
        gyro_int = gyro_int + beta;
        last_gyro_int = gyro_int;

        int_dt = round(integral_dt*1e6); %转化成为微秒;
        last_int_dt = int_dt;
        %reset
        alpha_first = 0;
        integrated_samples = 0;
        integral_dt = 0; 
        beta = 0;
        last_alpha = 0;

        int_flag = logical(true);
    end
end