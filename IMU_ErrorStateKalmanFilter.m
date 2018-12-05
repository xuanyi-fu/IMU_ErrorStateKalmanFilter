classdef IMU_ErrorStateKalmanFilter < handle
    
    properties  
        
        data_length
        
        nominalStates
        errorStates
        measurements
        noiseParam
        currentState
        
        ReferenceStates
        
    end
    methods
        %Constructor
        function obj = IMU_ErrorStateKalmanFilter(dataName)
            readInput(obj,dataName);
        end
        
        %Read input from data
        function readInput(obj,dataName)
            data = importdata(dataName);
            obj.data_length = size(data,1);
            %initialize arrays of states and measurements
            obj.nominalStates = NominalState(obj.data_length);
            obj.errorStates = ErrorState(obj.data_length);
            obj.measurements = Measurement(obj.data_length);
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
        function initializeStates(obj)
            p
        end
    end
end

