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
                %NOTICE THIS IS HARD-CODE NOW, BUT WILL UPDATE LATER
                obj.measurement(i).dt = 0.02;
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
    
        function eulerAngles = measurement2EulerAngles(obj,measurement)
        eulerAngles = zeros(3,1);
        eulerAngles(1) = atan2(-measurement.acc(2),-measurement.acc(3));
        eulerAngles(2) = asin(measurement.acc(1));
        eulerAngles(3) = atan2(-measurement.mag(2)*cos(eulerAngles(1))+...
            measurement.mag(3)*sin(eulerAngles(1)),...
            measurement.mag(1)*cos(eulerAngles(2))+...
            measurement.mag(2)*sin(eulerAngles(2))*sin(eulerAngles(1))+...
            measurement.mag(3)*sin(eulerAngles(2))*cos(eulerAngles(1)))-obj.noiseParam.mag_declination*pi/180;       
        end
    
        function quaternion = measurement2Quaternion(obj,measurement)
            quaternion = Quaternion(obj.measurement2EulerAngles(measurement));
        end
        
        %State propagation
        function nominalStatePrediction(obj)
            obj.nominalStates(obj.currentState)
            obj.nominalStates(obj.currentState+1)

            omega1 = obj.measurements(obj.currentState).av;
            omega2 = obj.measurements(obj.currentState+1).av;
            dt = obj.measurements(obj.currentState).dt;
            qnow = obj.nominalStates(obj.currentState).attitude;
            
            obj.nominalStates(obj.currentState+1).attitude = Integeration('zerothOrder_mid',dt,qnow,omega1,omega2);
            obj.nominalStates(obj.currentState+1).omega_b=obj.nominalStates(obj.currentState+1).omgea_b; 
        
        end
        %Integration Methods
        function q2 = Integeration(method,dt,q,omega1,varargin)
            if(strcmp(method,'zerothOrder_forward') && nargin == 3)
                q2 = q*Quaternion(omega1*dt);
            elseif(strcmp(method,'zerothOrder_backrward')&& nargin == 3)
                q2 = q*Quaternion(omega1*dt);
            elseif(strcmp(method,'zerothOrder_mid')&& nargin == 4)
                q2 = q*Quaternion((omega1+varargin{1})*dt/2);
            elseif(strcmp(method,'firstOrder')&& nargin == 4)
                q2 = q*(Quaterion((omega1+varargin{1})*dt/2)+...
                    Quaternion(dt*dt/24*[0;cross(omega1*dt,varargin{1}*dt)]));
            else
                error ('Integeration, Invalid Input Argument');
            end
        end
    end
    
end

