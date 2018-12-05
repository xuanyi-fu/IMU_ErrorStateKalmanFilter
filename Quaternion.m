classdef Quaternion
    %QUATERNION Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        s;
        v;
    end
    
    methods
        function obj = Quaternion(s,v)
            obj.s = s;
            obj.v = v;
        end
        
    end
end

