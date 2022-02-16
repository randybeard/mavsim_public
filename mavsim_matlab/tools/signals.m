% signals 
%   - step, impulse, etc.
%
% mavsim_matlab 
%     - Beard & McLain, PUP, 2012
%     - Last updated:  
%         2/12/2019 - RWB
classdef signals < handle
   %--------------------------------
    properties
        amplitude
        frequency
        period
        start_time
        duration
        dc_offset
        last_switch
    end
    %--------------------------------
    methods
        %------constructor-----------
        function self = signals(amplitude, frequency, start_time, dc_offset)
            self.amplitude = amplitude;
            self.frequency = frequency;  
            self.period = 2*pi/self.frequency;
            self.start_time = start_time;
            self.duration = 0.01;
            self.dc_offset = dc_offset;
            self.last_switch = start_time;
        end
        %---------------------------
        function y=step(self, time)
            % step function
            if time >= self.start_time
                y = self.amplitude;
            else
                y = 0.0;
            end
            y = y + self.dc_offset;
        end
        %---------------------------
        function y = sinusoid(self, time)
            % sinusoidal function
            if time >= self.start_time
                y = self.amplitude*sin(self.frequency * time);
            else
                y = 0.0;
            end
            y = y + self.dc_offset;
        end
        %---------------------------
        function y = square(self, time)
            % square wave function
            if time < self.start_time
                y = 0.0;
            elseif time < self.last_switch + self.period / 2.0
                y = self.amplitude;
            else
                y = -self.amplitude;
            end
            if time >= self.last_switch + self.period
                self.last_switch = time;
            end
            y = y + self.dc_offset;
        end
        %---------------------------
        function y = sawtooth(self, time)
            % sawtooth wave function
            if time < self.start_time
                y = 0.0;
            else
                y = self.amplitude * (time-self.last_switch);
            end
            if time >= self.last_switch + self.period
                self.last_switch = time;
            end
            y = y + self.dc_offset;
        end
        %---------------------------
        function y = impulse(self, time)
            % impulse function
            if (time >= self.start_time)... 
               & (time <= self.start_time+self.duration)
                y = self.amplitude;
            else
                y = 0.0;
            end
            y = y + self.dc_offset;
        end
        %---------------------------
        function y = doublet(self, time)
            % doublet function
            if (time >= self.start_time)...
               & (time < self.start_time + self.duration)
                y = self.amplitude;
            elseif (time >= self.start_time + self.duration)...
                   & (time <= self.start_time + 2*self.duration)
                y = -self.amplitude;
            else
                y = 0.0;
            end
            y = y + self.dc_offset;
        end
        %---------------------------
        function y = random(self, time)
            % random function
            if (time >= self.start_time)
                y = self.amplitude * randn;
            else
                y = 0.0;
            end
            y = y + self.dc_offset;
        end
    end
end