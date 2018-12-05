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
                obj.attitude = [0;0;0;0];
            %Array Constructor
            elseif nargin == 1
                obj(varargin{1},1)=obj;
            else
                obj.omega_b = varargin{1};
                obj.attitude = varargin{2};               
            end

        end
    end
end

