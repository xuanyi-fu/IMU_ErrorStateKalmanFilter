classdef ErrorState
    
    properties
        delta_theta
        delta_omega_b
        P
    end
    
    methods
        function obj = ErrorState(varargin)
            if nargin == 0
            obj.delta_theta = [0;0;0];
            obj.delta_omega_b = [0;0;0];
            %Array Constructor
            elseif nargin == 1
                obj(varargin{1},1)=obj;
            else
                obj.delta_theta = varargin{1};
                obj.delta_omega_b = varargin{2};        
            end
        end
        %set functions 
        function obj = set.delta_theta(obj,value)
            if(isvector(value) && size(value,1) == 3)
                obj.delta_theta = value;
            else
                error('delta_theta of the Nominal State Must be a [3x1] Vector')
            end
        end
        function obj = set.delta_omega_b(obj,value)
            if(isvector(value) && size(value,1) == 3)
                obj.delta_omega_b = value;
            else
                error('delta_omega_b of the Nominal State Must be a [3x1] Vector')
            end
        end
        function obj = set.P(obj,value)
            if(ismatrix(value) && size(value,1) == 6 && size(value,2) == 6)
%                 matrixPSD = value;
%                 for r = 1:size(matrixPSD,1)
%                     for c = 1:size(matrixPSD,2)
%                         if r == c
%                             matrixPSD(r,c) = abs(matrixPSD(r,c));
%                         else
%                             offDiagElement = mean([matrixPSD(r,c),matrixPSD(c,r)]);
%                             matrixPSD(c,r) = offDiagElement;
%                             matrixPSD(r,c) = offDiagElement;
%                         end
%                     end
%                 end
%                 obj.P = matrixPSD;
                obj.P = value;
            else
                error('delta_omega_b of the Nominal State Must be a [6x6] Symmertic Matrix')
            end
        end
    end 
end


