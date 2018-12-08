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
        %% Constructor & Main Loop
 
        function obj = IMU_ErrorStateKalmanFilter(dataName,noiseParameters)
            readInput(obj,dataName);
            obj.noiseParam = noiseParameters;
            initializeStates(obj);
            %MAIN LOOP
            %Nominal State Prediction
            tic
            while(obj.currentState < obj.data_length - 1)
                nominalStatePrediction(obj);
                errorStatePrediction(obj);
                errorStateCorrection(obj);
                injectErrorToNominal(obj);
                errorStateReset(obj);
                obj.currentState = obj.currentState + 1;
            end     
            toc
        end
        
        %% Import Data
        function readInput(obj,dataName)
            data            = importdata(dataName);
            data            = data(1:10000,:);
            obj.data_length = size(data,1);
            %initialize arrays of states and measurements
            obj.nominalStates = NominalState(obj.data_length);
            obj.errorStates   = ErrorState(obj.data_length);
            obj.measurements  = Measurement(obj.data_length);
            %build measurements with inputdata
            for i = 1:obj.data_length
                obj.measurements(i).acc = data(i,9:11).';
                obj.measurements(i).av  = data(i,27:29).'; 
                obj.measurements(i).mag = data(i,15:17).';
                %NOTICE THIS IS HARD-CODE NOW, BUT WILL UPDATE LATER
                obj.measurements(i).dt = 0.02;
            end
            %set current state to be one
            obj.currentState = 1;
        end  
        %% Initialize States
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
            initializeErrorStates(obj);
        end
        %% Nominal State prediction
        function nominalStatePrediction(obj)

            obj.nominalStates(obj.currentState+1).attitude = ...
                obj.nominalStatePredictionIntegeration('firstOrder');
            obj.nominalStates(obj.currentState+1).omega_b=...
                obj.nominalStates(obj.currentState).omega_b; 
        
        end
        
        %Integration Methods For Nominal State Prediction
        function q_next = nominalStatePredictionIntegeration(obj,method)
            %move the bias out of current and next omega before
            %the integeration
            omega_current = obj.measurements(obj.currentState).av-obj.nominalStates(obj.currentState).omega_b;
            omega_next = obj.measurements(obj.currentState+1).av-obj.nominalStates(obj.currentState+1).omega_b;
            
            dt = obj.measurements(obj.currentState).dt;
            q_current = obj.nominalStates(obj.currentState).attitude;
            
            %Decide which method of Integeration being used
            %Jose, Page 46 - 50
            %firstOrde method will take the change of axis into account
            if strcmp(method,'zerothOrder_forward')
                q_next = q_current*Quaternion(omega_current*dt);
            elseif strcmp(method,'zerothOrder_backrward')
                q_next = q_current*Quaternion(omega_current*dt);
            elseif strcmp(method,'zerothOrder_mid')
                q_next = q_current*Quaternion((omega_current+omega_next)*dt/2);
            elseif strcmp(method,'firstOrder')
                q_next = q_current*(Quaternion((omega_current+omega_next)*dt/2)+...
                    Quaternion(dt*dt/24*[0;cross(omega_current*dt,omega_next*dt)]));
            else
                error ('method integeration, Invalid Input Argument');
            end
        end
        %% Error State Prediction
         function errorStatePrediction(obj) 
            % The Prediction of delta_theta
            % Jose, Page58
            % First compute (w_m-w_b)*delta_t
            % Then make it to a rotation matrix using matrix exponential
            % map.
            
            dt = obj.measurements(obj.currentState).dt;
            omega_m = obj.measurements(obj.currentState).av;
            omega_b = obj.nominalStates(obj.currentState).omega_b;
            R = expm(v2s((omega_m - omega_b)*dt));
            
            %%% Notice that the following two predictions will always be
            %%% zeros if you initilize the errorStates to be zero!
            
            %Predict delta_theta
            obj.errorStates(obj.currentState+1).delta_theta = ...
                R.'*obj.errorStates(obj.currentState).delta_theta-...
                obj.errorStates(obj.currentState).delta_omega_b*dt;
            
            %Predict delta_omega_b
            obj.errorStates(obj.currentState+1).delta_omega_b = ...
                obj.errorStates(obj.currentState).delta_omega_b;
            
            %Update the Covariance
            %First we construct Fx
            %Jose Page 59
            Fx = [R.',-eye(3)*dt;
                zeros(3),eye(3)];

            obj.errorStates(obj.currentState+1).P = ...
                Fx*obj.errorStates(obj.currentState).P*Fx.'+...
                obj.noiseParam.getQi(dt);
          end
          %% Error State Correction

         function errorStateCorrection(obj)
             %Calculate H and detZ = y-h(det_x)
             [H,detZ] = calH(obj,obj.nominalStates(obj.currentState+1).attitude.q,  obj.measurements(obj.currentState+1));
             P = obj.errorStates(obj.currentState+1).P;
             % Calculate Kalman gain
             K = (P*H') / ( H*P*H' + obj.noiseParam.V); 
             % State correction
             delta_x =  K * detZ;
             obj.errorStates(obj.currentState+1).delta_theta = delta_x(1:3);
             obj.errorStates(obj.currentState+1).delta_omega_b = delta_x(4:6);
             % Covariance correction
             obj.errorStates(obj.currentState+1).P = P -  K *(H*P*H'+ obj.noiseParam.V)*K';
         end
         %Calculate Measurement Jacobian
         %Error Need to be Checked
         function [H,detZ] = calH(obj,q,measurements)
             % Normalise magnetometer measurement
             if(norm(measurements.mag) == 0), return; end	% 
             measurements.mag = measurements.mag / norm(measurements.mag);	% normalise magnitude,very important!!!!
             % Normalise accelerometer measurement
             if(norm(measurements.acc) == 0), return; end	% handle NaN
             measurements.acc  = measurements.acc / norm(measurements.acc);	% normalise accelerometer ,very important!!!!
             % Reference direction of Earth's magnetic feild
             h = quaternProd(q, quaternProd([0; measurements.mag], quatInv(q)));
             b = [0 norm([h(2) h(3)]) 0 h(4)];
             
             Ha = [2*q(3),                 	-2*q(4),                    2*q(1),                         -2*q(2)
                 -2*q(2),                 	-2*q(1),                   -2*q(4),                         -2*q(3)
                 0,                         4*q(2),                    4*q(3),                         0];
             
             Hm = [-2*b(4)*q(3),                2*b(4)*q(4),               -4*b(2)*q(3)-2*b(4)*q(1),       -4*b(2)*q(4)+2*b(4)*q(2)
                 -2*b(2)*q(4)+2*b(4)*q(2),	   2*b(2)*q(3)+2*b(4)*q(1),    2*b(2)*q(2)+2*b(4)*q(4),       -2*b(2)*q(1)+2*b(4)*q(3)
                 2*b(2)*q(3),                2*b(2)*q(4)-4*b(4)*q(2),	   2*b(2)*q(1)-4*b(4)*q(3),        2*b(2)*q(2)];       
             
             Hx = [Ha, zeros(3,3)
                 Hm, zeros(3,3)];
             %Hx = [Ha, zeros(3,3)];
             Q_detTheta  = [-q(2),    -q(3),      -q(4)
                 q(1),    -q(4),       q(3) 
                 q(4),     q(1),      -q(2) 
                 -q(3),     q(2),       q(1)];
      
             Xx = [0.5*Q_detTheta , zeros(4,3)
                 zeros(3)       , eye(3)];
      
             H = Hx*Xx; 
      
             detZ_a = [ 2*(q(2)*q(4)  - q(1)*q(3)) + measurements.acc(1)
                 2*(q(1)*q(2) + q(3)*q(4)) + measurements.acc(2)
                 2*(0.5 - q(2)^2 - q(3)^2) + measurements.acc(3)];
             detZ_m =[((2*b(2)*(0.5 - q(3)^2 - q(4)^2) + 2*b(4)*(q(2)*q(4) - q(1)*q(3))) + measurements.mag(1))
                 ((2*b(2)*(q(2)*q(3) - q(1)*q(4)) + 2*b(4)*(q(1)*q(2) + q(3)*q(4))) + measurements.mag(2))
                 ((2*b(2)*(q(1)*q(3) + q(2)*q(4)) + 2*b(4)*(0.5 - q(2)^2 - q(3)^2)) + measurements.mag(3))]; 
      
             detZ   = [detZ_a;detZ_m];
         end
         %% Inject Error into nominal State
         function injectErrorToNominal(obj)
         % inject the attitude error
         obj.nominalStates(obj.currentState+1).attitude = ...
             obj.nominalStates(obj.currentState+1).attitude*...
             Quaternion(obj.errorStates(obj.currentState+1).delta_theta);
         % inject the bias error
         obj.nominalStates(obj.currentState+1).omega_b = ...
             obj.nominalStates(obj.currentState+1).omega_b+...
             obj.errorStates(obj.currentState+1).delta_omega_b;
         end
         %% ESKF reset
         function errorStateReset(obj)
             obj.errorStates(obj.currentState+1).delta_theta = [0;0;0];
             obj.errorStates(obj.currentState+1).delta_omega_b = [0;0;0];
            %obj.errorStates(obj.currentState+1).P = obj.errorStates(obj.currentState+1).P
         end
         %% Plot figure
         function plot(obj)
             eulerAngles = zeros(3,obj.data_length - 1);
             for i = 1:obj.data_length-1
                 eulerAngles(:,i)=obj.nominalStates(i).attitude.toEulerAngles();
             end             
             figure('Name','IMU Error State Kalman Filter For Attitude Estimation')
             subplot(1,3,1)
             plot(eulerAngles(1,:))
             title('Roll')
             subplot(1,3,2)
             plot(eulerAngles(2,:))
             title('Pitch')
             subplot(1,3,3)
             plot(eulerAngles(3,:))
             title('Yaw')    
         end
         %% Helper Functions
    
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
            quaternion = Quaternion(obj.measurement2EulerAngles(measurement),'euler2Quaternion');
        end         
         
    end
end

