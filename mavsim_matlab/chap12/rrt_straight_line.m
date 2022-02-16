% rrt straight line path planner for mavsim_matlab
%
% mavsim_matlab 
%     - Beard & McLain, PUP, 2012
%     - Last updated:  
%         4/8/2019 - RWB
classdef rrt_straight_line < handle
   %--------------------------------
    properties
        segment_length
        tree
    end
    %--------------------------------
    methods
        %------constructor-----------
        function self = rrt_straight_line
            % standard length of path segments
            self.segment_length = 300;
            % waypoints definition
            addpath('../message_types'); 
            self.tree = msg_waypoints();
            self.tree.type = 'fillet';
        end
        %------methods-----------
        function waypoints = update(self, start_pos, end_pos, Va, map, radius)
            
            % add the start node to the tree
            self.tree.add(start_pos, Va, inf, 0, 0, 0);
            % add(ned_position, airspeed, course, cost, flag_connect_to_goal)

            % check to see if start_node connects directly to end_node
            if ( (self.distance(start_pos, end_pos)<self.segment_length )...
                    && (self.collision(start_pos, end_pos, map)==0) )
                self.tree.add(end_pos, Va, inf, self.distance(start_pos, end_pos), 1, 1);
            else
                numPaths = 0;
                while numPaths<3
                    flag = self.extend_tree(end_pos, Va, map);
                    numPaths = numPaths + flag;
                end
            end

            % find path with minimum cost to end_node
            waypoints_not_smooth = self.find_minimum_path(end_pos);
            waypoints = self.smooth_path(waypoints_not_smooth, map);
            self.plot_map(map, waypoints_not_smooth, waypoints, radius);
        end
        %----------------------------------------
        function d=distance(self, pos_s, pos_e)
        end
        %----------------------------------------
        function collision_flag = collision(self, pos_s, pos_e, map)
        end
        %----------------------------------------
        function points = points_along_path(self, pos_s, pos_e, Del)
            %   Find points along straight-line path separted by Del (to be used in
            %   collision detection)
            points = pos_s;
            q = [pos_e-pos_s];
            L = norm(q);
            q = q/L;

            w = pos_s;
            for i=2:floor(L/Del)
                w = w + Del*q;
                points = [points, w];
            end
        end
        %----------------------------------------
        function h_agl = height_above_ground(self, map, point)
        end
        %----------------------------------------
        function flag = extend_tree(self, end_pos, Va, map)
        end
        %----------------------------------------
        function config=random_configuration(self, map, pd)        
        end
        %----------------------------------------
        function waypoints = find_minimum_path(self, end_node)
        end
        %----------------------------------------
        function smooth_waypoints = smooth_path(self, waypoints, map)
        end
        %----------------------------------------
        function self = plot_map(self, map, waypoints, smoothed_waypoints, radius)
            addpath('../chap12'); world_view = world_viewer();
            S = 2000;
            figure(3); clf;
            world_view.drawMap(map);
            hold on
            world_view.drawNonSmoothedWaypoints(waypoints, radius);
            world_view.drawWaypoints(smoothed_waypoints, radius);
            world_view.drawTree(self.tree, radius);
            
            title('UAV')
            xlabel('East')
            ylabel('North')
            zlabel('-Down')
            axis([-S/5,S,-S/5,S,0,3*map.building_max_height]);
            view(-40,70)  % set the view angle for figure
            axis square;
            grid on
        end
    end
end