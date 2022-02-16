% msg_autopilot
%   - message type for commands sent to the autopilot
%
% mavsim_matlab 
%     - Beard & McLain, PUP, 2012
%     - Update history:  
%         2/12/2019 - RWB
classdef msg_autopilot
   %--------------------------------
    properties
        airspeed_command
        course_command
        altitude_command
        phi_feedforward
    end
    %--------------------------------
    methods
        %------constructor-----------
        function self = msg_autopilot()
            self.airspeed_command = 0;
            self.course_command = 0;
            self.altitude_command = 0;
            self.phi_feedforward = 0;
        end
    end
end