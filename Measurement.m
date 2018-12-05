classdef Measurement
    
    properties
        acc
        av %angular_velocity
        mag
    end
    
    methods
        %Constructor
        function obj = Measurement(varargin)
            if nargin == 0
                obj.av = [0;0;0];
                obj.acc = [0;0;0];
                obj.mag = [0;0;0];
            %Array Constructor
            elseif nargin == 1
                obj(varargin{1},1)=obj;
            else
                obj.acc = varargin{1};
                obj.av =  varargin{2};
                obj.mag = varargin{3};             
            end
        end
        %Set Methods for properties
        function obj = set.av(obj,value)
            obj.av = value;
        end
        function obj = set.acc(obj,value)
            obj.acc = value;
        end
        function obj = set.mag(obj,value)
            obj.mag = value;
        end        
    end
end