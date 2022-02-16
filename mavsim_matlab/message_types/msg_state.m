% msg_state
%   - message type for state to be passed between different blocks in
%   architecture
%
% mavsim_matlab 
%     - Beard & McLain, PUP, 2012
%     - Update history:  
%         12/19/2018 - RWB
classdef msg_state
   %--------------------------------
    properties
        pn        % north position in meters
        pe        % east position in meters
        h         % altitude in meters
        phi       % roll angle in radians
        theta     % pitch angle in radians
        psi       % yaw angle in radians
        Va        % airspeed in m/s
        alpha     % angle of attack in radians
        beta      % side slip angle in radians
        p         % roll rate in radians/sec
        q         % pitch rate in radians/sec
        r         % yaw rate in radians/sec
        Vg        % ground speed in meters/sec
        chi       % course angle in radians
        gamma     % flight path angle in radians
        wn        % wind in the north direction in meters/sec
        we        % wind in the east direction in meters/sec
        bx        % gyro bias for p in radians/sec
        by        % gyro bias for q in radians/sec
        bz        % gyro bias for r in radians/sec
    end
    %--------------------------------
    methods
        %------constructor-----------
        function self = msg_state()
            self.pn = 0;
            self.pe = 0;
            self.h = 0;
            self.phi = 0;
            self.theta = 0;
            self.psi = 0;
            self.Va = 0;
            self.alpha = 0;
            self.beta = 0;
            self.p = 0;
            self.q = 0;
            self.r = 0;
            self.Vg = 0;
            self.chi = 0;
            self.gamma = 0;
            self.wn = 0;
            self.we = 0;
            self.bx = 0;
            self.by = 0;
            self.bz = 0;
        end
    end
end