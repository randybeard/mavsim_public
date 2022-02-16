% alpha_filter
%
% mavsim_matlab 
%     - Beard & McLain, PUP, 2012
%     - Last updated:  
%         3/2/2019 - RWB
classdef alpha_filter < handle
   %--------------------------------
    properties
        alpha
        y
    end
    %--------------------------------
    methods
        %------constructor-----------
        function self = alpha_filter(alpha)
            self.alpha = alpha;
            self.y = 0;
        end
        %------methods-----------
        function y = update(self, u)
            self.y = 
            y = self.y;
        end
    end
end