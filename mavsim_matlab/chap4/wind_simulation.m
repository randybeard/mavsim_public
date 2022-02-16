% wind Simulation - simulates steady state and wind gusts
%
% mavMatSim 
%     - Beard & McLain, PUP, 2012
%     - Update history:  
%         12/27/2018 - RWB
classdef wind_simulation < handle
   %--------------------------------
    properties
        steady_state
        A
        B
        C
        gust_state
        gust_
        Ts
    end
    %--------------------------------
    methods
        %------constructor-----------
        function self = wind_simulation(Ts)
            self.steady_state = 
            self.A = 
            self.B = 
            self.C = 
            self.gust_state = 
            self._gust = [0; 0; 0];
            self.Ts = Ts;
        end
        %---------------------------
        function wind=update(self)
            wind = [self.steady_state; self._gust];
        end
        %----------------------------
        function self = gust(self)
            w = randn;
            self.gust_state = self.gust_state + self.Ts*(A*self.gust_state + B*w);
            self._gust = self.C*self.gust_state;
        end
    end
end