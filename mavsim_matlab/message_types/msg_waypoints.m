% msg_waypoints
%   - message type for path planning
%
% mavsim_matlab 
%     - Beard & McLain, PUP, 2012
%     - Update history:  
%         4/8/2019 - RWB
classdef msg_waypoints < handle
   %--------------------------------
    properties
        flag_waypoints_changed
        flag_manager_requests_waypoints
        type
        max_waypoints
        num_waypoints
        ned
        airspeed
        course
        cost
        parent
        flag_connect_to_goal
    end
    %--------------------------------
    methods
        %------constructor-----------
        function self = msg_waypoints()
            % the first two flags are used for interacting with the path
            % planner
            %
            % flag to indicate waypoints recently changed (set by planner)
            self.flag_waypoints_changed = 1;  
            
            % type of waypoint following:
            %   - straight line following
            %   - fillets between straight lines
            %   - follow dubins paths
            self.type = 'straight_line';
            %self.type = 'fillet';
            %self.type = 'dubins';
            % current number of valid waypoints in memory
            self.num_waypoints = 0;
            % [n, e, d] - coordinates of waypoints
            self.ned = [];
            % the airspeed that is commanded along the waypoints
            self.airspeed = [];
            % the desired course at each waypoint (used only for Dubins
            % paths)
            self.course = [];
            
            % these last three variables are used by the path planner
            % running cost at each node
            self.cost = [];
            % index of the parent to the node
            self.parent = [];   
            % can this node connect to the goal?  1==connected, 0==not
            % connected
            self.flag_connect_to_goal = [];  
        end
        %---------------------------------------------------
        function self = add(self, ned, airspeed, course,...
                            cost, parent, connect_to_goal)
            self.num_waypoints = self.num_waypoints + 1;
            self.ned = [self.ned, ned];  % ned is 3x1 position vector
            self.airspeed = [self.airspeed, airspeed];
            self.course = [self.course, course];
            self.cost = [self.cost, cost];
            self.parent = [self.parent, parent];
            self.flag_connect_to_goal = [self.flag_connect_to_goal, connect_to_goal];
        end
    end
end