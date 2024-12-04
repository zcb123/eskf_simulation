classdef AlphaFilter < handle
    properties
        	cutoff_freq;
	        alpha;
	        filter_state;
    
    end
    methods 
        
        function obj = AlphaFilter(alpha,cutoff_freq)
            obj.alpha = alpha;
            obj.cutoff_freq = cutoff_freq;
            obj.filter_state = zeros(3,1);
        end

        function obj = reset_filter(obj,sample)
            obj.filter_state = sample;
        end

        function res = update(obj,sample)
            [row,col]=size(sample);
            if row < 3
                col = 2;
            end
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


