classdef IMU_ErrorStateKalmanFilter < handle
    
    properties  
        
        data_length
        noiseParam
        
        nominalStates
        errorStates
        measurements
        
        
        currentState
        ReferenceStates
        
    end
    methods
        %Constructor
        function obj = IMU_ErrorStateKalmanFilter(dataName,noiseParameters)
            readInput(obj,dataName);
            obj.noiseParam = noiseParameters;
            initializeStates(obj);
        end
        
        %Read input from data
        function readInput(obj,dataName)
            data            = importdata(dataName);
            obj.data_length = size(data,1);
            %initialize arrays of states and measurements
            obj.nominalStates = NominalState(obj.data_length);
            obj.errorStates   = ErrorState(obj.data_length);
            obj.measurements  = Measurement(obj.data_length);
            %build measurements with inputdata
            for i = 1:obj.data_length
                obj.measurements(i).acc = data(i,9:11);
                obj.measurements(i).av  = data(i,27:29); 
                obj.measurements(i).mag = data(i,15:17);
            end
            %set current state to be one
            obj.currentState = 1;
        end  
        
        %Compute the initial state
        function initializeNominalStates(obj)
            obj.nominalStates(1).attitude = obj.measurement2Quaternion(obj.measurements(1));
            obj.nominalStates(1).omega_b  = obj.noiseParam.omega_b_init;
        end
        
        function initializeErrorStates(obj)
            obj.errorStates(1).P             = obj.noiseParam.P_init;
            obj.errorStates(1).delta_theta   = [0;0;0];
            obj.errorStates(1).delta_omega_b = [0;0;0];
        end
        function initializeStates(obj)
            initializeNominalStates(obj);
            initializeErrorStates(obj)
        end
        
    end
    
    methods(Static)
    %Extract Euler Angles from measurement
    function eulerAngles = measurement2EulerAngles(measurement)
        eulerAngles = zeros(3,1);
        %Roll
        eulerAngles(1) = atan2(measurement.acc(2),measurement.acc(3));
        eulerAngles(2) = asin(measurement.acc(1));
        eulerAngles(3) = atan2(-measurement.mag(2)*cos(eulerAngles(1))+...
            measurement.mag(3)*sin(eulerAngles(1)),...
            measurement.mag(1)*cos(eulerAngles(2))+...
            measurement.mag(2)*sin(eulerAngles(2))*sin(eulerAngles(1))+...
            measurement.mag(3)*sin(eulerAngles(2))*cos(eulerAngles(1)));       
    end
    
    function quaternion = measurement2Quaternion(measurement)
        quaternion = Quaternion(IMU_ErrorStateKalmanFilter.measurement2EulerAngles(measurement));
    end
    
    end
end

