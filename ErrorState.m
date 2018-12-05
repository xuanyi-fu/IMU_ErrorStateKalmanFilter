classdef ErrorState
    
    properties
        delta_theta
        delta_omega_b
    end
    
    methods
        function obj = ErrorState(varargin)
            if nargin == 0
            obj.delta_theta = [0;0;0];
            obj.delta_omega_b = [0;0;0];
            %Array Constructor
            elseif nargin == 1
                obj(varargin{1},1)=obj;
            else
                obj.delta_theta = varargin{1};
                obj.delta_omega_b = varargin{2};        
            end
        end
    end
end

