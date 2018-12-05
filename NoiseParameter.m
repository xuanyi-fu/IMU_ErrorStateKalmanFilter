classdef NoiseParameter
    
    properties
        Q %covariance matrix of the pertubation impulses, Jose, page 59
        V %covariance matrix of the measurements, Jose, page 61
        omega_b_init %initial bias of the gyrometer
        omega_variance_init %initial variance of bias of the gyrometer
        attitude_variance_init %initial variance of attitude
        P_init % initla P matrix of error-state
    end
    
    methods
        function obj = NoiseParameter(gyroVar,gyroBias,gyroBiasVar,gyroBiasVarInit,...
                accVar,MagVar,attVarInit)
        if nargin == 7
            if isvector(gyroVar) && size(gyroVar,1) == 3 &&...
               isvector(gyroBiasVar) && size(gyroBiasVar,1) == 3 &&...
               isvector(accVar) && size(accVar,1) == 3 &&...
               isvector(MagVar) && size(MagVar,1) == 3 &&...
               isvector(gyroBias) && size(gyroBias,1) == 3&&...
               isvector(gyroBiasVarInit) && size(gyroBiasVarInit,1) == 3&&...
               isvector(attVarInit) && size(attVarInit,1) == 3
               
            obj.Q = diag([gyroVar.',gyroBiasVar.']);
            obj.V = diag([accVar.',MagVar.']);
            obj.omega_b_init = gyroBias;
            obj.omega_variance_init = gyroBiasVarInit;
            obj.attitude_variance_init = attVarInit;
            obj.P_init = diag([obj.attitude_variance_init.',obj.omega_variance_init.']);
            else
                error('All Variance&Bias Params Must be [3x1] Vectors')
            end
        elseif nargin == 1 && isa(gyroVar,'NoiseParameter')
            obj = gyroVar;
        else
            error ('Invalid Input Params For NoiseParameter Constructor')
        end
        end
    end
end

