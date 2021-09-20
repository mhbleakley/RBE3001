classdef Traj_Planner
    %TRAJ_PLANNER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        positionArray; % n by 3 array 
    end
    
    methods
        function obj = Traj_planner(inputArg1,inputArg2)
            %TRAJ_PLANNER Construct an instance of this class
            %   Detailed explanation goes here
            obj.Property1 = inputArg1 + inputArg2;
        end
        
        function a = cubic_traj(self, ti, tf, vi, vf, posi, posf)
            M = [1 ti ti^2 ti^3;
                0 1 2*ti 3*(ti^2);
                1 tf tf^2 tf^3;
                0 1 2*tf 3*(tf^2)];
            
            givenVals = [posi; vi; posf; vf];
            
            a = M\givenVals;
%             disp(a);
        end
        
        function C = cubic_polynomial(self, a0, a1, a2, a3, t)
            C = a0 + a1*t + a2*(t^2) + a3*(t^3);
        end
    end
end

