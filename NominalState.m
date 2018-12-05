classdef NominalState
    %NOMINALSTATE Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        attitude %Quaternion, the attitude of IMU.
        omega_b  %Vector [3x1], the bias of the gyrometer.
    end
    
    methods
        function obj = NominalState(varargin)
            if nargin == 0
                obj.omega_b = [0;0;0];
                obj.attitude = Quaternion([0;0;0;0]);
            %Array Constructor
            elseif nargin == 1
                obj(varargin{1},1)=obj;
            else
                obj.omega_b = varargin{1};
                obj.attitude = varargin{2};               
            end
        %Set Methods for properties
        end
        function obj = set.attitude(obj,value)
            if(isa(value,'Quaternion'))
                %make sure the attitude of the IMU is a UNIT Quaternion
                obj.attitude = normalize(value);
            else
                error('the attitude of the Nominal State Must be a Quaternion')
            end
        end
        function obj = set.omega_b(obj,value)
            if(isvector(value) && size(value,1) == 3)
                obj.omega_b = value;
            else
                error('the angular velocity bias of the Nominal State Must be a [3x1] Vector')
            end
        end
        end
end



