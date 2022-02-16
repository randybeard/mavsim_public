% dubins_parameters
%   - Dubins parameters that define path between two configurations
%
% mavsim_matlab 
%     - Beard & McLain, PUP, 2012
%     - Update history:  
%         3/25/2019 - RWB
classdef dubins_parameters < handle
   %--------------------------------
    properties
        p_s
        chi_s
        p_e
        chi_e
        radius
        length
        center_s
        dir_s
        center_e
        dir_e
        r1
        r2
        r3
        n1
        n3
    end
    %--------------------------------
    methods
        %------constructor-----------
        function self = dubins_parameters()
                self.p_s = inf*[1;1;1];  % the start position in re^3
                self.chi_s = inf; % the start course angle
                self.p_e = inf*[1;1;1]; %the end position in re^3
                self.chi_e = inf; % the end course angle
                self.radius = inf; % turn radius
                self.length = inf; % length of the Dubins path
                self.center_s = inf*[1;1;1]; % center of the start circle
                self.dir_s = inf; % direction of the start circle
                self.center_e = inf*[1;1;1]; %center of the end circle
                self.dir_e = inf; % direction of the end circle
                self.r1 = inf*[1;1;1]; % vector in re^3 defining half plane H1
                self.r2 = inf*[1;1;1]; % vector in re^3 defining position of half plane H2
                self.r3 = inf*[1;1;1]; % vector in re^3 defining position of half plane H3
                self.n1 = inf*[1;1;1]; % unit vector in re^3 along straight line path
                self.n3 = inf*[1;1;1]; % unit vector defining direction of half plane H3
        end
        %------external methods------
        function self = update(self, ps, chis, pe, chie, R)

                self.p_s = 
                self.chi_s = 
                self.p_e =  
                self.chi_e =  
                self.radius =  
                self.length =  
                self.center_s =  
                self.dir_s =  
                self.center_e =  
                self.dir_e = 
                self.r1 =  
                self.n1 = 
                self.r2 =  
                self.r3 = 
                self.n3 =  
            end
        end
    end
end