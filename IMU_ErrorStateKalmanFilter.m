classdef IMU_ErrorStateKalmanFilter < handle
    
    properties       
        data_length
        noiseParam
        
        nominalStates
        errorStates
        measurements
        
        currentState
        referenceStates    
    end
    
    properties(Constant)
        RAD2DEG = 180/pi;
    end
    methods
        %% Constructor & Main Loop
 
        function obj = IMU_ErrorStateKalmanFilter(dataName,noiseParameters)
            obj.noiseParam = noiseParameters;
            readInput(obj,dataName);
            
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
            obj.data_length = size(data,1);
            %initialize arrays of states and measurements
            obj.nominalStates = NominalState(obj.data_length);
            obj.errorStates   = ErrorState(obj.data_length);
            obj.measurements  = Measurement(obj.data_length);
            %obj.referenceStates = Quaternion(obj.data_length);
            obj.referenceStates = zeros(obj.data_length,3);
            %build measurements with inputdata
            for i = 1:obj.data_length
                obj.measurements(i).acc = data(i,9:11).';
                obj.measurements(i).av  = data(i,27:29).'; 
                obj.measurements(i).mag = data(i,15:17).';
                %obj.referenceStates(i) = Quaternion(data(i,30:32).','euler2Quaternion');
                obj.referenceStates(i,1)=data(i,30)*obj.RAD2DEG;
                obj.referenceStates(i,2)=data(i,31)*obj.RAD2DEG;
                obj.referenceStates(i,3)=data(i,32)*obj.RAD2DEG;
                %NOTICE THIS IS HARD-CODE NOW, BUT WILL UPDATE LATER
                obj.measurements(i).dt = 0.02;
            end
            %Ref
            
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
            % J.S., Page58
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
             %First we compute the H due to the gravity field of the earth
             q_vector     = obj.nominalStates(obj.currentState+1).attitude.q;
             q_quat = obj.nominalStates(obj.currentState+1).attitude;
             %Notice that if you want to fit this program with your data,
             %your IMU must measure the gravity as [0;0;+g] when heading up.
             %Assume the gravity field on the earth is [0;0;g] everywhere.
             
             d_gravity = [0;0;1];
             d_magnet  = [26790.5;-4245.4;46873.2];
             d_magnet = d_magnet/norm(d_magnet);
             
             %NOTICE THAT THIS MAGNET FIELD IS HARD-CODE HERE
             %FIND YOUR MAGFIELD VECTOR HERE: https://www.ngdc.noaa.gov/geomag/calculators/
             Ha = computeJacobian(obj,q_vector,d_gravity);
             Hm = computeJacobian(obj,q_vector,d_magnet);
             %These zeros are due to the bias of the angular velocity
             %cannot be measured. Therefore, in the h(x) function, there is
             %no terms having relationship with omega_b. Thus, H(:,4:6) are
             %all zeros
             H_delta_x = [Ha zeros(3,3);
                  Hm zeros(3,3)];
   
             %Next we have to compute X_delta_x
             %First we compute Q_delta_theta
             %J.S. Page 62
             %Q_delta_theta = (1/2)*q_quat.leftProductMatrix()*[0 0 0;ones(3)];
             %Or you can compute the Q_delta_theta explicitly and copy it
             %here
             Q_delta_theta  = 0.5*[-q_vector(2),    -q_vector(3),     -q_vector(4)
                                    q_vector(1),    -q_vector(4),      q_vector(3) 
                                    q_vector(4),    q_vector(1),      -q_vector(2) 
                                   -q_vector(3),    q_vector(2),       q_vector(1)];
             
             X_delta_x =  [Q_delta_theta , zeros(4,3)
                           zeros(3)       , eye(3)];
                       
             %After all the preparation above, we can compute H now.
             H = H_delta_x*X_delta_x;
             
             %Then we compute the gain matrix 'K'
             %J.S. Page 61
             P = obj.errorStates(obj.currentState+1).P;
             V = obj.noiseParam.V;
             K = P*H.'/(H*P*H+V);

             %Next we compute h(x)
             %h(x) is computed by the following method
             %First we use the conjugation property of quaternions to make
             %the vector describe in the earth frame to be described in the
             %IMU's Frame
             
             hx_gravity = h(obj,q_quat,Quaternion([0;d_gravity]));
             hx_magnet  = h(obj,q_quat,Quaternion([0;d_magnet]));
             %Then we combine them together to be a [6x1] vector and
             %compute delta_x_hat = K( y - h ( x_hat ) )
             hx = [hx_gravity;hx_magnet];
             
             y = [obj.measurements(obj.currentState+1).acc;...
                  obj.measurements(obj.currentState+1).mag ];
             delta_x_hat = K*(y-hx);
             %Then we make the correction of error state
             obj.errorStates(obj.currentState+1).delta_theta   = delta_x_hat(1:3,1);
             obj.errorStates(obj.currentState+1).delta_omega_b = delta_x_hat(4:6,1);
             %Finally we can update the covariance
             obj.errorStates(obj.currentState+1).P = P - K * (H * P * H'+ V) * K.';
         end
         %Calculate Measurement Jacobian
         %Error Need to be Checked        
         function jacobian = computeJacobian(obj,q,d)
             dx = d(1);
             dy = d(2);
             dz = d(3);
             q1 = q(1);
             q2 = q(2);
             q3 = q(3);
             q4 = q(4);
             
             jacobian = [2*dy*q4-2*dz*q3 2*dy*q3+2*dz*q4         -4*dx*q3+2*dy*q2-2*dz*q1 -4*dx*q4+2*dy*q1+2*dz*q2;
                        -2*dx*q4+2*dz*q2 2*dx*q3-4*dy*q2+2*dz*q1 2*dx*q2+2*dz*q4          -2*dx*q1-4*dy*q4+2*dz*q3;
                        2*dx*q3-2*dy*q2  2*dx*q4-2*dy*q1-4*dz*q2 2*dx*q1+2*dy*q4-4*dz*q3  2*dx*q2+2*dy*q3          ];
         end
         
         function h = h(obj,q,d)
             %q and d are all quaternions
             %we first compute the pure quaternion(quaternions only have 
             %the vector part) h_quaternion
             h_quaternion = q'*d*q;
             %Then we return its vector part
             h = h_quaternion.q;
             h = h(2:4);
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
         % Plot figure
         function plotEulerAngleErrors(obj)
             eulerAngles = zeros(3,obj.data_length - 1);
             eulerAngleErrors = zeros(3,obj.data_length - 1);
             for i = 1:obj.data_length-1
                 eulerAngles(:,i)=obj.nominalStates(i).attitude.toEulerAngles();
                 eulerAngleErrors(:,i)=eulerAngles(:,i)-obj.referenceStates(i,:).';
             end             
             figure('Name','IMU Error State Kalman Filter For Attitude Estimation')
             subplot(2,3,1)
             plot(eulerAngles(1,:))
             hold on
             plot(obj.referenceStates(:,1))
             title('Roll')
             subplot(2,3,2)
             plot(eulerAngles(2,:))
             hold on
             plot(obj.referenceStates(:,2))
             title('Pitch')
             subplot(2,3,3)
             plot(eulerAngles(3,:))
             hold on
             plot(obj.referenceStates(:,3))
             title('Yaw')  
             subplot(2,3,4)
             plot(eulerAngleErrors(1,:))
             subplot(2,3,5)
             plot(eulerAngleErrors(2,:))
             subplot(2,3,6)
             plot(eulerAngleErrors(3,:))
        end
%         function plotNormErrors(obj)
%             errors = zeros(obj.data_length - 1);
%             for i = 1:10:obj.data_length - 1
%                 errors(i)=norm(obj.referenceStates(i).q - ...
%                     obj.nominalStates(i).attitude.q);
%             end
%             figure('Name','IMU Error State Kalman Filter For Attitude Estimation')
%             plot(errors)
%         end
         %% Helper Functions
    
        function eulerAngles = measurement2EulerAngles(obj,measurement)
        eulerAngles = zeros(3,1);
        eulerAngles(1) = atan2(measurement.acc(2),measurement.acc(3));
        eulerAngles(2) = -asin(measurement.acc(1));
        eulerAngles(3) = atan2(-measurement.mag(2)*cos(eulerAngles(1))+...
            measurement.mag(3)*sin(eulerAngles(1)),...
            measurement.mag(1)*cos(eulerAngles(2))+...
            measurement.mag(2)*sin(eulerAngles(2))*sin(eulerAngles(1))+...
            measurement.mag(3)*sin(eulerAngles(2))*cos(eulerAngles(1)))-obj.noiseParam.mag_declination/180*pi;       
        end
    
        function quaternion = measurement2Quaternion(obj,measurement)
            quaternion = Quaternion(obj.measurement2EulerAngles(measurement),'euler2Quaternion');
        end         
         
    end
end

