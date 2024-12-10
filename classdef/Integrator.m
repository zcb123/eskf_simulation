classdef Integrator<handle
    properties
        alpha;
        last_val;
        integral_dt
        reset_interval_min
        reset_samples_min
        integrated_samples

    end

    properties(Constant)
        DT_MIN = 1.17549435e-38;
        DT_MAX = 0.65535;
    end

    methods
        function obj = Integrator()
            obj.alpha = [0 0 0];
            obj.last_val = [0 0 0];
            obj.integral_dt = 0;
            obj.reset_interval_min = 0.00375;
            obj.reset_samples_min = 8;
            obj.integrated_samples = 0;
        end
        %val 为列向量
        function obj = put(obj,val,dt)
            if (dt > obj.DT_MIN) && (obj.integral_dt + dt < obj.DT_MAX)
			    obj.alpha = obj.alpha + obj.integrate(val, dt);

		    else 
			    reset();
			    obj.last_val = val;
		    end
        end
        function res = integrate(obj,val,dt)

            obj.integrated_samples = obj.integrated_samples + 1;
		    obj.integral_dt = obj.integral_dt + dt;
		    delta_alpha = (val + obj.last_val) *dt * 0.5;
		    obj.last_val = val;
   
		    res = delta_alpha;
        
        end
        function [integral,integral_dt,ret] = get_val_reset(obj)
            if (obj.integral_ready()) 
			    integral = obj.alpha;
			    integral_dt = round(obj.integral_dt * 1e6); 
			    obj.reset();
    
			    ret = true;
            else
                integral = [0 0 0];
                integral_dt = obj.integral_dt;
                ret = false;
            end
	
        end
        function obj = reset(obj)
            obj.alpha = [0 0 0];
            obj.integral_dt = 0;
            obj.integrated_samples = 0;
        end
        function ret = integral_ready(obj)
            ret = (obj.integrated_samples >= obj.reset_samples_min) || (obj.integral_dt >= obj.reset_interval_min);
        end
    end


end