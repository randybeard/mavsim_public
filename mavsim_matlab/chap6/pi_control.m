% pi_control
%
% mavsim_matlab 
%     - Beard & McLain, PUP, 2012
%     - Last updated:  
%         2/13/2019 - RWB
classdef pi_control < handle
   %--------------------------------
    properties
        kp
        ki
        Ts
        limit
        integrator
        error_delay_1
    end
    %--------------------------------
    methods
        %------constructor-----------
        function self = pi_control(kp, ki, Ts, limit)
            self.kp = kp;
            self.ki = ki;
            self.Ts = Ts;
            self.limit = limit;
            self.integrator = 0;
            self.error_delay_1 = 0;
        end
        %----------------------------
        function u_sat = update(self, y_ref, y)
        end
        %----------------------------
        function out = saturate(self, in)
            % saturate u at +- self.limit
            if in >= self.limit
                out = self.limit;
            elseif in <= -self.limit
                out = -self.limit;
            else
                out = in;
            end
        end
    end
end