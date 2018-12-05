classdef ErrorState
    
    properties
        delta_theta
        delta_omega_b
    end
    
    methods
        function obj = ErrorState(delta_theta,delta_omega_b)
            obj.delta_theta = delta_theta;
            obj.delta_omega_b = delta_omega_b;
        end
    end
end

