classdef NoiseParameter
    
    properties
        %Qi %covariance matrix of the pertubation impulses, Jose, page 59
        %Qi will be built using the following two properties with different
        %dt
        sigma_omega_n_square
        sigma_omega_bn_square
        V %covariance matrix of the measurements, Jose, page 61
        omega_b_init %initial bias of the gyrometer
        omega_variance_init %initial variance of bias of the gyrometer
        attitude_variance_init %initial variance of attitude
        P_init % initla P matrix of error-state
        mag_declination
    end
    
    methods
        function obj = NoiseParameter(gyroVar,gyroBias,gyroBiasVar,gyroBiasVarInit,...
                accVar,MagVar,attVarInit,magDec)
        if nargin == 8
            if isvector(gyroVar) && size(gyroVar,1) == 3 &&...
               isvector(gyroBiasVar) && size(gyroBiasVar,1) == 3 &&...
               isvector(accVar) && size(accVar,1) == 3 &&...
               isvector(MagVar) && size(MagVar,1) == 3 &&...
               isvector(gyroBias) && size(gyroBias,1) == 3&&...
               isvector(gyroBiasVarInit) && size(gyroBiasVarInit,1) == 3&&...
               isvector(attVarInit) && size(attVarInit,1) == 3&&...
               isscalar(magDec)
            obj.sigma_omega_n_square = gyroVar;
            obj.sigma_omega_bn_square = gyroBiasVar;
%             obj.Q = diag([gyroVar.',gyroBiasVar.']);
            obj.V = diag([accVar.',MagVar.']);
            obj.omega_b_init = gyroBias;
            
            obj.attitude_variance_init = attVarInit;
            obj.omega_variance_init = gyroBiasVarInit;
            
            obj.P_init = diag([obj.attitude_variance_init.',obj.omega_variance_init.']);
            obj.mag_declination = magDec;
            else
                error('All Variance&Bias Params Must be [3x1] Vectors')
            end
        elseif nargin == 1 && isa(gyroVar,'NoiseParameter')
            obj = gyroVar;
        else
            error ('Invalid Input Params For NoiseParameter Constructor')
        end
        end
        
        function Qi = getQi(obj,dt)
            Qi = diag([obj.sigma_omega_n_square.'*dt,obj.sigma_omega_bn_square.'*dt]);
        end
    end
end

