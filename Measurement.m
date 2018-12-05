classdef Measurement
    %MEASUREMENT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        av %angular_velocity
        acc
        mag
        
    end
    
    methods
        function obj = measurement(av,acc,mag)
            obj.av = av;
            obj.acc = acc;
            obj.mag = mag;
        end
    end
end