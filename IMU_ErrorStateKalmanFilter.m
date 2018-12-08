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
                obj.measurements(i).dt = 0.02;
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
        
        %Nominal State prediction
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
        
        %Error State Prediction
         function errorStatePrediction(obj) 
             %recall variables
            obj.errorStates(obj.currentState)
            obj.errorStates(obj.currentState+1)
            
             %Define a constant
             wb=obj.nominalStates(obj.currentState).omega_b;
             %Calculate F
             Fx = calcFx( wb, obj.measurements(obj.currentState));
             det_x = [obj.errorStates(obj.currentState).det_theta;obj.errorStates(obj.currentState).delta_omega_b];
             det_x_pre = Fx * det_x;                         %this prediction always be zero
             
             obj.errorStates(obj.currentState+1).det_theta = det_x_pre(1:3);
             obj.errorStates(obj.currentState+1).delta_omega_b = det_x_pre(4:6);
             
             %Prediction Covariance
             %Define constant matrixs
             Qi = obj.noiseParam.Q * obj.measurements(obj.currentState).dt;
             Fi = eye(6);
             
             obj.errorStates(obj.currentState+1).P = Fx *  obj.errorStates(obj.currentState).P * Fx' + Fi * Qi * Fi'; %how to remove the Nan ????
             %errorState_pre.P = enforcePSD(errorState_pre.P);%this is important,it removes the NAN problem successfully~
         end
         %Calculate Fx
         function Fx = calcFx(wb, Measurement)
             % Multiplies the error state in the linearized continuous-time
             % error state model
             Fx = zeros(6,6);
             omegaHat = Measurement.av - wb; % need to be checked whether it is a 3x1 vector
             Fx(1:3,1:3) = axisAngleToRotMat(omegaHat*Measurement.dt)';
             Fx(1:3,4:6) = -eye(3)*Measurement.dt;
             Fx(4:6,1:3) = zeros(3);
             Fx(4:6,4:6) = eye(3);
         end
         %Transfor Quaternion into Rotation Matrix
         function R=axisAngleToRotMat(q)
             qw=q(1);qx=q(2);qy=q(3);qz=q(4);
             R=[qw^2+qx^2-qy^2-qz^2, 2*(qx*qy-qw*qz), 2*(qx*qz+qw*qy); ...
                 2*(qx*qy+qw*qz), qw^2-qx^2+qy^2-qz^2, 2*(qz*qy-qw*qx);...
                 2*(qx*qz-qw*qy), 2*(qz*qy+qw*qx), qw^2-qx^2-qy^2+qz^2];
         end
         
         
         %Error State Correction
         function errorStateCorrection(obj)
             %Calculate H and detZ = y-h(det_x)
             [H,detZ] = calH(obj.nominalStates(obj.currentState+1).attitude,  obj.measurements(obj.currentState+1));
             P = obj.errorStates(obj.currentState+1).P;
             % Calculate Kalman gain
             K = (P*H') / ( H*P*H' + obj.noiseParam.V); 
             % State correction
             det_x =  K * detZ;
             obj.errorStates(obj.currentState+1).det_theta = det_x(1:3);
             obj.errorStates(obj.currentState+1).det_omega_b = det_x(4:6);
             % Covariance correction
             obj.errorStates(obj.currentState+1).P = P -  K *(H*P*H'+ obj.noiseParam.V)*K';
         end
         %Calculate Measurement Jacobian
         %Error Need to be Checked
         function [H,detZ] = calH(q,measurements)
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
         
    end
    
end

