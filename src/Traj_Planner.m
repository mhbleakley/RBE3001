classdef Traj_Planner
    %TRAJ_PLANNER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        positionArray; % n by 3 array 
    end
    
    methods
        function obj = Traj_planner(positionArray1)
            %TRAJ_PLANNER Construct an instance of this class
            %   Detailed explanation goes here
            obj.positionArray = positionArray1;
        end
        
        function a = quintic_traj(self, posi, posf, ti, tf, ai, af, vi, vf)
            M = [1 ti ti^2 ti^3 ti^4 ti^5;
                0 1 2*ti 3*(ti^2) 4*(ti^3) 5*(ti^4);
                0 0 2 6*ti 12*(ti^2) 20*(ti^3);
                1 tf tf^2 tf^3 tf^4 tf^5;
                0 1 2*tf 3*(tf^2) 4*(tf^3) 5*(tf^4);
                0 0 2 6*tf 12*(tf^2) 20*(tf^3)]
            
            givenVals = [posi; vi; ai; posf; vf; af];
            
            a = M\givenVals;
        end
        
         function a = cubic_traj(self, ti, tf, vi, vf, posi, posf)
            M = [1 ti ti^2 ti^3;
                0 1 2*ti 3*(ti^2);
                1 tf tf^2 tf^3;
                0 1 2*tf 3*(tf^2)];
            
            givenVals = [posi; vi; posf; vf];
            
            a = M\givenVals;
         end
         
         
         
         function a = cubic_poly_traj(self, ti, tf, vi, vf, posi, posf, time)
            x = self.cubic_traj(ti, tf, vi, vf, posi, posf);
            a = self.cubic_polynomial(x, time);
         end 
%          function a = linear_trajectory(self, ti, tf, posi, posf)
%             M = [1 ti;
%                 1 tf];
%             
%             givenVals = [posi; posf];
%             
%             a = M\givenVals;
%         end
%          
        function C = cubic_polynomial(self, a, t)
            C = a(1,1) + t * a(2,1) + (t^2) * a(3,1) + (t^3) * a(4,1);
        end
        
        
        function C = quintic_polynomial(self, a, t)
            C = a(1,1) + t * a(2,1) + (t^2) * a(3,1) + (t^3) * a(4,1) + (t^4) * a(5,1) + (t^5) * a(6,1);
        end
        
       function a = linear_traj(self,posi, posf, ti, tf, timestep) %ti/f = s, timestep = ms
           traj = self.cubic_traj(ti, tf, 0, 0, posi, posf);
           a = self.cubic_polynomial(traj,  timestep);
       end
       
       
      function a = quintic_linear_traj(self,posi, posf, ti, tf, timestep) %ti/f = s, timestep = ms
           traj = self.quintic_traj(posi, posf, ti, tf, 0, 0, 0, 0);
           a = self.quintic_polynomial(traj,  timestep);
      end
        
      
        

    end
end

