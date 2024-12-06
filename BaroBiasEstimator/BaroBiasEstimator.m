classdef BaroBiasEstimator<handle
    properties

        status; 
        
        state;
	    state_max;
	    state_drift_rate; %/< in m/s
	    dt;
    
	    gate_size; %/< Used for innovation filtering (innovation test ratio)
	    state_var; %/< Initial state uncertainty variance (m^2)
	    process_var; %/< State process noise variance (m^2/s^2)
	    state_var_max; %/< Used to rain the state variance (m^2)
    
	    signed_innov_test_ratio_lpf; %/< innovation sequence monitoring; used to detect a bias in the state
    
	    lpf_time_ant;
        process_var_boost_gain;

    end

    methods
        function obj = BaroBiasEstimator()
            obj.signed_innov_test_ratio_lpf = AlphaFilter(0,0);
            obj.status = struct('bias',0,'bias_var',0,'innov',0,'innov_var',0,'innov_test_ratio',0);
            obj.state = 0.;
            obj.state_max = 100.;
            obj.state_drift_rate = 0.005; %/< in m/s
            obj.dt = 0.01;
        
            obj.gate_size = 3; %/< Used for innovation filtering (innovation test ratio)
            obj.state_var = 0.1; %/< Initial state uncertainty variance (m^2)
            obj.process_var = 25.0e-6; %/< State process noise variance (m^2/s^2)
            obj.state_var_max = 2; %/< Used to rain the state variance (m^2)
            obj.lpf_time_ant = 10;
            obj.process_var_boost_gain = 1e3;
        end
  
        function obj = predict(obj,dt)
    
            global FLT_EPSILON;
    
            obj.state_var = obj.state_var +obj.process_var * dt * dt;
    
	        rainStateVar();
        
	        if (dt > FLT_EPSILON && fabsf(obj.dt - dt) > 0.001) 
		        obj.signed_innov_test_ratio_lpf.setParameters(dt, obj.lpf_time_ant);
		        obj.dt = dt;
            end
    
        end
    
        function obj = rainStateVar(obj)
    
            obj.state_var = saturation(obj.state_var, 1e-8, obj.state_var_max);
        
        end
    
    
	    function obj = fuseBias(obj,measurement, measurement_var)
            
                innov_var = obj.state_var + measurement_var;
	            innov = measurement - obj.state;
	            K = obj.state_var / innov_var;
	            innov_test_ratio = obj.computeInnovTestRatio(innov, innov_var);
            
	            if (isTestRatioPassing(innov_test_ratio)) 
		            obj.updateState(K, innov);
		            obj.updateStateCovariance(K);
            
	            end
            
	            if (obj.isLargeOffsetDetected()) 
		            % A bias in the state has been detected by the innovation
		            % sequence check.
		            obj.bumpStateVariance();
	            end
            
	            signed_innov_test_ratio = sign(innov) * innov_test_ratio;
	            obj.signed_innov_test_ratio_lpf.update(saturation(signed_innov_test_ratio, -1, 1));
            
	            obj.status = obj.packStatus(innov, innov_var, innov_test_ratio);
        end
    
    
        function obj = setBias(obj,bias)  
            obj.state = bias; 
        end
    
    
    
        function obj = setProcessNoiseStdDev(obj,process_noise)
	    
		    obj.process_var = process_noise * process_noise;
		    obj.state_drift_rate = 3*process_noise;
    
	    end
	    function obj = setBiasStdDev(state_noise)  
    
            obj.state_var = state_noise * state_noise; 
        
        end
	    function obj = setInnovGate(gate_size) 
            obj.gate_size = gate_size; 
        end
    
	    function obj = setMaxStateNoise(max_noise) 
            obj.state_var_max = max_noise * max_noise; 
        end
    
        function val = getBias(obj) 
            val = obj.state; 
        end
    
        function val = getBiasVar() 
            val = obj.state_var; 
        end
    
        function res = getStatus()    
            res = obj.status; 
        end
    
	    
    
        function val = computeInnovTestRatio(innov, innov_var) 
            val = innov * innov / (obj.gate_size * obj.gate_size * innov_var);
        end
    
    
        function ret = isTestRatioPassing(innov_test_ratio) 
            ret= innov_test_ratio<1;
        end
    
        function obj = updateState(obj,K, innov)
            obj.state = saturation(obj.state + K * innov, -obj.state_max, obj.state_max);
        end
    
        function obj = updateStateCovariance(obj,K)
            obj.state_var =obj.state_var - K * obj.state_var;
	        constrainStateVar();
        end
    
        function ret = isLargeOffsetDetected(obj) 
            
            ret = fabsf(obj.signed_innov_test_ratio_lpf.getState()) > 0.2;
        
        end
	    
        
        function obj = bumpStateVariance(obj)
            obj.state_var = obj.state_var + obj.process_var_boost_gain * obj.process_var * obj.dt * obj.dt;
    
        end
	    function res = packStatus(obj,innov, innov_var, innov_test_ratio) 
            obj.status.bias = obj.state;
            obj.status.bias_var = obj.state_var;
            obj.status.innov = innov;
            obj.status.innov_var = innov_var;
            obj.status.innov_test_ratio = innov_test_ratio;
            res = obj.status;
        end
    end
end