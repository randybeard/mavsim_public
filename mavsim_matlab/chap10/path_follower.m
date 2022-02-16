% path follower for mavsim_matlab
%
% mavsim_matlab 
%     - Beard & McLain, PUP, 2012
%     - Last updated:  
%         3/12/2019 - RWB
classdef path_follower < handle
   %--------------------------------
    properties
        chi_infty
        k_path
        k_orbit
        gravity
        commands
    end
    %--------------------------------
    methods
        %------constructor-----------
        function self = path_follower
            self.chi_infty = 
            self.k_path = 
            self.k_orbit = 
            self.gravity = 9.8;
            addpath('../message_types'); 
            self.commands = msg_autopilot();
        end
        %------methods-----------
        function autopilot_commands = update(self, path, state)
            if isequal(path.flag, 'line')
                self.follow_straight_line(path, state);
            else % path.flag == 'orbit'
                self.follow_orbit(path, state);
            end
            % return the estimated state
            autopilot_commands = self.commands;
        end
        %---------------------------
        function self = follow_straight_line(self, path, state)
            self.commands.airspeed_command = 
            self.commands.course_command = 
            self.commands.altitude_command = 
            self.commands.phi_feedforward = 
        end
        %---------------------------
        function self = follow_orbit(self, path, state)
            self.commands.airspeed_command = 
            self.commands.course_command = 
            self.commands.altitude_command = 
            self.commands.phi_feedforward = 
        end
        %---------------------------
        function chi_c = wrap(self, chi_c, chi)
            while chi_c-chi > pi
                chi_c = chi_c - 2*pi;
            end
            while chi_c-chi < -pi
                chi_c = chi_c + 2*pi;
            end
        end
    end
end