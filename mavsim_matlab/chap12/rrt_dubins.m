% rrt dubins path planner for mavsim_matlab
%
% mavsim_matlab 
%     - Beard & McLain, PUP, 2012
%     - Last updated:  
%         4/10/2019 - RWB
classdef rrt_dubins < handle
   %--------------------------------
    properties
        segment_length
        tree
        dubins_path
    end
    %--------------------------------
    methods
        %------constructor-----------
        function self = rrt_dubins
            % standard length of path segments
            self.segment_length = 600;
            % waypoints definition
            addpath('../message_types'); 
            self.tree = msg_waypoints();
            self.tree.type = 'dubins';
            addpath('../chap11')
            self.dubins_path = dubins_parameters();
        end
        %------methods-----------
        function waypoints = update(self, start_pose, end_pose, Va, map, radius)
            % add the start node to the tree
            self.tree.add(start_pose(1:3), Va, start_pose(4), 0, 0, 0);
            % add(ned_position, airspeed, course, cost, flag_connect_to_goal)

            % check to see if start_node connects directly to end_node
            if ( (self.distance(start_pose, end_pose)<self.segment_length )...
                    && (self.collision(start_pose, end_pose, map, radius)==0)...
                    && (self.distance(start_pose, end_pose)>=2*radius) )
                self.tree.add(end_pos, Va, inf, self.distance(start_pos, end_pos), 1, 1);
            else
                numPaths = 0;
                while numPaths<3
                    flag = self.extend_tree(end_pose, Va, map, radius);
                    numPaths = numPaths + flag;
                end
            end

            % find path with minimum cost to end_node
            waypoints_not_smooth = self.find_minimum_path(end_pose);
            waypoints = self.smooth_path(waypoints_not_smooth, map, radius);
            self.plot_map(map, waypoints_not_smooth, waypoints, radius);
        end
        %----------------------------------------
        function d=distance(self, start_pose, end_pose)
        end
        %----------------------------------------
        function collision_flag = collision(self, pose_s, pose_e, map, radius)
        end
        %----------------------------------------
        function points = points_along_path(self, Del)
            %   Find points along Dubins path separted by Del (to be used in
            %   collision detection)
            points = [];
            % points along start circle
            th1 = mod(atan2(self.dubins_path.p_s(2)-self.dubins_path.center_s(2),...
                            self.dubins_path.p_s(1)-self.dubins_path.center_s(1)), 2*pi);
            th2 = mod(atan2(self.dubins_path.r1(2)-self.dubins_path.center_s(2),...
                            self.dubins_path.r1(1)-self.dubins_path.center_s(1)), 2*pi);
            if self.dubins_path.dir_s>0
                if th1>=th2
                    th = [th1:Del:2*pi,0:Del:th2];
                else
                    th = [th1:Del:th2];
                end
            else
                if th1<=th2
                    th = [th1:-Del:0,2*pi:-Del:th2];
                else
                    th = [th1:-Del:th2];
                end
            end
            for i=1:length(th)
                new_point = [...
                    self.dubins_path.center_s(1)+self.dubins_path.radius*cos(th(i));...
                    self.dubins_path.center_s(2)+self.dubins_path.radius*sin(th(i));...
                    self.dubins_path.center_s(3);...
                    ];
                points = [points, new_point];
            end
  
            % points along straight line 
            sig = 0;
            while sig<=1
                new_point = [...
                    (1-sig)*self.dubins_path.r1(1) + sig*self.dubins_path.r2(1);...
                    (1-sig)*self.dubins_path.r1(2) + sig*self.dubins_path.r2(2);...
                    (1-sig)*self.dubins_path.r1(3) + sig*self.dubins_path.r2(3);...
                    ];
                points = [points, new_point];
                sig = sig + Del;
            end
    
            % points along end circle
            th2 = mod(atan2(self.dubins_path.p_e(2)-self.dubins_path.center_e(2),...
                            self.dubins_path.p_e(1)-self.dubins_path.center_e(1)), 2*pi);
            th1 = mod(atan2(self.dubins_path.r2(2)-self.dubins_path.center_e(2),...
                            self.dubins_path.r2(1)-self.dubins_path.center_e(1)), 2*pi);
            if self.dubins_path.dir_e>0
                if th1>=th2
                    th = [th1:Del:2*pi,0:Del:th2];
                else
                    th = [th1:Del:th2];
                end
            else
                if th1<=th2
                    th = [th1:-Del:0,2*pi:-Del:th2];
                else
                    th = [th1:-Del:th2];
                end
            end
            for i=1:length(th)
                new_point = [...
                    self.dubins_path.center_e(1)+self.dubins_path.radius*cos(th(i));...
                    self.dubins_path.center_e(2)+self.dubins_path.radius*sin(th(i));...
                    self.dubins_path.center_e(3);...
                    ];
                points = [points, new_point];
            end
        end
        %----------------------------------------
        function h_agl = height_above_ground(self, map, point)
            %   finds altitude above ground level
        end
        %----------------------------------------
        function flag = extend_tree(self, end_pose, Va, map, radius)
        end
        %----------------------------------------
        function pose=random_pose(self, map, pd)        
        end

        %----------------------------------------
        function waypoints = find_minimum_path(self, end_node)
        end
        %----------------------------------------
        function smooth_waypoints = smooth_path(self, waypoints, map, radius)
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
            axis([-S/5,S+S/5,-S/5,S+S/5,0,3*map.building_max_height]);
            view(-40,70)  % set the view angle for figure
            axis square;
            grid on
        end
    end
end