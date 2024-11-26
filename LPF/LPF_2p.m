classdef LPF_2p < handle
    properties
        b0;
        b1;
        b2;
        a0;
        a1;
        a2;
        delay_element_0;
        delay_element_1;
        delay_element_2;
    end
    methods(Access=public)

        function obj = LPF_2p(sample_freq,cutoff_freq)

            fr = sample_freq / cutoff_freq;
            ohm = tan(pi / fr);
            c = 1 + 2* cos(pi / 4) * ohm + ohm * ohm;
            
            obj.b0 = ohm * ohm / c;
            obj.b1 = 2 * obj.b0;
            obj.b2 = obj.b0;

            obj.a0 = 1;
            obj.a1 = 2 * (ohm * ohm - 1) / c;
            obj.a2 = (1 - 2 * cos(pi / 4) * ohm + ohm * ohm) / c;
            obj.delay_element_1 = [0 0 0];
            obj.delay_element_2 = [0 0 0];
        end

        function [obj,output] = apply(obj,sample)

            obj.delay_element_0 = sample - obj.delay_element_1*obj.a1 - obj.delay_element_2*obj.a2;
            output = obj.delay_element_0*obj.b0 + obj.delay_element_1*obj.b1 + obj.delay_element_2*obj.b2;
            obj.delay_element_2 = obj.delay_element_1;
            obj.delay_element_1 = obj.delay_element_0;

        end
    end
end