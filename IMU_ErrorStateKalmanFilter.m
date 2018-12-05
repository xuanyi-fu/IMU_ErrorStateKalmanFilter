classdef IMU_ErrorStateKalmanFilter
    
    properties  
        
        data
        data_length
        
        nominalStates
        errorStates
        measurements
        noiseParam
        currentState
        
        ReferenceStates
        
    end
    
    methods
        function obj = IMU_ErrorStateKalmanFilter(dataName)
            obj.data = importdata(dataName);
            obj.data_length = size(obj.data,1);
            %initialize arrays of states and measurements
            obj.nominalStates = NominalState.empty(obj.data_length,1);
            obj.errorStates = ErrorState.empty(data_length,1);
            obj.measurements = Measurement.empty(data_length,1);
            %build measurements with inputdata
            for i = 1:obj.data_length
                obj.measurements(i)=Measurement(data(i,1:3),data(i,4:6),data(i,7:9));
            end
            %set current state to be one
            obj.currentState = 1;
        end
    end
end

