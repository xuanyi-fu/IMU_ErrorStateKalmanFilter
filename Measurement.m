classdef Measurement
    
    properties
        acc
        av %angular_velocity
        mag
        dt
    end
    
    methods
        %Constructor
        function obj = Measurement(varargin)
            if nargin == 0
                obj.av = [0;0;0];
                obj.acc = [0;0;0];
                obj.mag = [0;0;0];
                obj.dt = 0;
            %Array Constructor
            elseif nargin == 1
                obj(varargin{1},1)=obj;
            else
                obj.acc = varargin{1};
                obj.av =  varargin{2};
                obj.mag = varargin{3};
                obj.dt = varargin{4};
            end
        end
        %Set Methods for properties
        function obj = set.av(obj,value)
            obj.av = value;
        end
        function obj = set.acc(obj,value)
            obj.acc = value/norm(value);
        end
        function obj = set.mag(obj,value)
            obj.mag = value/norm(value);
        end    
        function obj = set.dt(obj,value)
            obj.dt = value;
        end 
    end
end