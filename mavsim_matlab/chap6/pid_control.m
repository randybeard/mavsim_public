% pid_control
%
% mavsim_matlab 
%     - Beard & McLain, PUP, 2012
%     - Last updated:  
%         2/13/2019 - RWB
classdef pid_control < handle
   %--------------------------------
    properties
        kp
        ki
        kd
        Ts
        limit
        integrator
        error_delay_1
        error_dot_delay_1
        a1
        a2
    end
    %--------------------------------
    methods
        %------constructor-----------
        function self = pid_control(kp, ki, kd, Ts, sigma, limit)
            self.kp = kp;
            self.ki = ki;
            self.kd = kd;
            self.Ts = Ts
            self.limit = limit;
            self.integrator = 0;
            self.error_delay_1 = 0;
            self.error_dot_delay_1 = 0;
            self.a1 = (2*sigma-Ts)/(2*sigma+Ts);
            self.a2 = 2/(2*sigma+Ts);
        end
        %----------------------------
        function u_sat = update(self, y_ref, y, reset_flag)
        end
        %----------------------------
        function u_sat = update_with_rate(self, y_ref, y, ydot, reset_flag)
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