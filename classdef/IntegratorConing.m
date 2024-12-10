classdef IntegratorConing<Integrator

    properties
        beta;
        last_delta_alpha;
        last_alpha;
    end

    methods
        function obj = IntegratorConing()
            obj.beta = [0 0 0];
            obj.last_alpha = [0 0 0];
            obj.last_delta_alpha = [0 0 0];
        end
         function obj = put(obj,val,dt)
            if ((dt > obj.DT_MIN) && (obj.integral_dt + dt < obj.DT_MAX)) 
			
			    delta_alpha = (obj.integrate(val, dt));
    
			    
			    obj.beta = obj.beta + cross((obj.last_alpha + obj.last_delta_alpha * (1/6)) , delta_alpha) * 0.5;
			    obj.last_delta_alpha = delta_alpha;
			    obj.last_alpha = obj.alpha;
    
			    
			    obj.alpha = obj.alpha + delta_alpha;

	        else 
		        obj.reset();
		        obj.last_val = val;
	        end
         end
         function [integral, integral_dt,flag] = get_val_reset(obj)

             [integral, integral_dt,flag] = get_val_reset@Integrator(obj);
             if flag
                integral = integral + obj.beta;
                obj.beta = 0;
                obj.last_alpha = 0;
                
             end
                
            
         end
         function reset(obj)
            reset@Integrator(obj);
            obj.beta = [0 0 0];
            obj.last_alpha = [0 0 0];
         end
    end
end

