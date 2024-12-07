classdef AlphaFilter_float < handle
    properties
        	cutoff_freq;
	        alpha;
	        filter_state;
    
    end
    methods 
        
        function obj = AlphaFilter_float(alpha,cutoff_freq)
            obj.alpha = alpha;
            obj.cutoff_freq = cutoff_freq;
            obj.filter_state = 0;
        end

        function obj = reset_filter(obj,sample)
            obj.filter_state = sample;
        end
        function obj = setParameters(sample_interval,time_constant)
            global FLT_EPSILON;
            denominator = time_constant + sample_interval;

		    if denominator > FLT_EPSILON
			    setAlpha(sample_interval / denominator);
            end

        end
        function obj = setAlpha(obj,alpha)
            obj.alpha = alpha;
        end
        function res = update(obj,sample)
%             [row,col]=size(sample);
%             if row < 3
%                 col = 2;
%             end
            obj.filter_state = updateCalculation(obj,sample);
            res = obj.filter_state;
        end

        function res = updateCalculation(obj,sample)
            res = (1-obj.alpha).*obj.filter_state+obj.alpha.*sample;
        end
        function res = getState(obj)
            res = obj.filter_state;
        end
    end
end


