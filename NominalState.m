classdef NominalState
    %NOMINALSTATE Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        attitude %Quaternion, the attitude of IMU.
        omega_b  %Vector [3x1], the bias of the gyrometer.
    end
    
    methods
        function obj = NominalState(attitude,omega_b)
            obj.omega_b = omega_b;
            obj.attitude = attitude;
        end
    end
end

