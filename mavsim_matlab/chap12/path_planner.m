% path planner for mavsim_matlab
%
% mavsim_matlab 
%     - Beard & McLain, PUP, 2012
%     - Last updated:  
%         4/2/2019 - RWB
classdef path_planner < handle
   %--------------------------------
    properties
        waypoints
        rrt_straight_line
        rrt_dubins
    end
    %--------------------------------
    methods
        %------constructor-----------
        function self = path_planner
            % waypoints definition
            addpath('../message_types'); 
            self.waypoints = msg_waypoints(); 
            addpath('../chap12')
            self.rrt_straight_line = rrt_straight_line();
            self.rrt_dubins = rrt_dubins();
        end
        %------methods-----------
        function waypoints = update(self, map, state, radius)
            % this flag is set for one time step to signal a redraw in the
            % viewer
            %planner_flag = 1;  % return simple waypoint path
            %planner_flag = 2;  % return dubins waypoint path
            %planner_flag = 3;  % plan path through city using straight-line RRT
            planner_flag = 4;  % plan path through city using dubins RRT
            disp('planning...');
            if planner_flag == 1
                Va = 25;
                self.waypoints.type = 'fillet';
                self.waypoints.add([0; 0; -100], Va, inf, inf, 0, 0);
                self.waypoints.add([1000; 0; -100], Va, inf, inf, 0, 0);
                self.waypoints.add([0; 1000; -100], Va, inf, inf, 0, 0);
                self.waypoints.add([1000; 1000; -100], Va, inf, inf, 0, 0);
            elseif planner_flag == 2
                Va = 25;
                self.waypoints.type = 'dubins';
                self.waypoints.add([0; 0; -100], Va, 0, inf, 0, 0);
                self.waypoints.add([1000; 0; -100], Va, 45*pi/180, inf, 0, 0);
                self.waypoints.add([0; 1000; -100], Va, 45*pi/180, inf, 0, 0);
                self.waypoints.add([1000; 1000; -100], Va, -135*pi/180, inf, 0, 0);
            elseif planner_flag == 3
                Va = 25;
                % start configuration is current configuration
                ps = [state.pn; state.pe; -state.h];
                % desired end configuration
                if norm(ps -[map.city_width; map.city_width; -state.h]) < map.city_width/2
                    pe = [0; 0; -state.h];
                else
                    pe = [map.city_width; map.city_width; -state.h];
                end
                self.waypoints.type = 'fillet';
                self.waypoints = self.rrt_straight_line.update(ps, pe, Va, map, radius);
            elseif planner_flag == 4
                Va = 25;
                % start pose is current pose
                start_pose = [state.pn; state.pe; -state.h; state.chi];
                % desired end pose
                if norm(start_pose(1:3) -[map.city_width; map.city_width; -state.h]) < map.city_width/2
                    end_pose = [0; 0; -state.h; 0];
                else
                    end_pose = [map.city_width; map.city_width; -state.h; 0];
                end
                self.waypoints.type = 'dubins';
                self.waypoints = self.rrt_dubins.update(start_pose, end_pose, Va, map, radius);
            else
                disp('Error in Path Planner: Undefined planner type.')
            end
            % return the planned waypoints
            waypoints = self.waypoints;
            disp('... done planning');
        end
    end
end