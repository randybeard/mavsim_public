% world viewer for mavsim_matlab
%
% mavsim_matlab 
%     - Beard & McLain, PUP, 2012
%     - Last updated:  
%         4/10/2019 - RWB
classdef world_viewer < handle
    properties
        mav_handle
    	mav_vertices
    	mav_faces
    	mav_facecolors
        path_handle
        waypoint_handle
        plot_initialized
        dubins_path
        map
    end
    %--------------------------------
    methods
        %------constructor-----------
        function self = world_viewer()
            self.mav_handle = [];
            self.path_handle = [];
            self.waypoint_handle = [];
            [self.mav_vertices, self.mav_faces, self.mav_facecolors] = self.define_mav();
            self.plot_initialized = 0;  
            addpath('../chap11');
            self.dubins_path = dubins_parameters();
        end
        %---------------------------
        function self=update(self, map, waypoints, path, state)
            
            S = 2000;
            if self.plot_initialized==0
                figure(1); clf;
                %scale = 4;
                self.drawBody(state.pn, state.pe, -state.h,...
                              state.phi, state.theta, state.psi);
                hold on
                self.drawWaypoints(waypoints, path.orbit_radius);
                self.drawPath(path, S);
                self.drawMap(map);
                title('UAV')
                xlabel('East')
                ylabel('North')
                zlabel('-Down')
                axis([-S/5,S+S/5,-S/5,S+S/5,0,3*map.building_max_height]);
                view(-40,70)  % set the view angle for figure
                axis square;
                grid on
                self.plot_initialized = 1;
            else
                self.drawBody(state.pn, state.pe, -state.h,... 
                              state.phi, state.theta, state.psi);
                 if path.flag_path_changed == 1
                    self.drawPath(path, S);
                 end
                 if waypoints.flag_waypoints_changed == 1
                    self.drawWaypoints(waypoints, path.orbit_radius);
                 end
            end
        end
        %---------------------------
        function self = drawBody(self, pn, pe, pd, phi, theta, psi)
            Vertices = self.rotate(self.mav_vertices, phi, theta, psi);   % rotate rigid body  
            Vertices = self.translate(Vertices, pn, pe, pd);     % translate after rotation
            % transform vertices from NED to XYZ (for matlab rendering)
            R = [...
                0, 1, 0;...
                1, 0, 0;...
                0, 0, -1;...
                ];
            Vertices = R*Vertices;
            if isempty(self.mav_handle)
                self.mav_handle = patch(...
                    'Vertices', Vertices',...
                    'Faces', self.mav_faces,...
                    'FaceVertexCData',self.mav_facecolors,...
                    'FaceColor','flat');
            else
                set(self.mav_handle,...
                    'Vertices',Vertices',...
                    'Faces',self.mav_faces);
                drawnow
            end
        end 
        %---------------------------
        function pts=rotate(self, pts, phi, theta, psi)
            % define rotation matrix (right handed)
            R_roll = [...
                        1, 0, 0;...
                        0, cos(phi), sin(phi);...
                        0, -sin(phi), cos(phi)];
            R_pitch = [...
                        cos(theta), 0, -sin(theta);...
                        0, 1, 0;...
                        sin(theta), 0, cos(theta)];
            R_yaw = [...
                        cos(psi), sin(psi), 0;...
                        -sin(psi), cos(psi), 0;...
                        0, 0, 1];
            R = R_roll*R_pitch*R_yaw;  
            % note that R above either rotates the reference frame for a
            % fixed vector or rotates the vector in a fixed reference frame
            % using with a left-handed rotation.  We want to rotate all
            % points in a right-handed rotation in a fixed reference frame, 
            % so we must transpose
            R = R';
            % rotate vertices
            pts = R*pts;
        end
        %---------------------------
        % translate vertices by pn, pe, pd
        function pts = translate(self, pts, pn, pe, pd)
            pts = pts + repmat([pn;pe;pd],1,size(pts,2));
        end
        %---------------------------
        function [V, F, colors] = define_mav(self)
        % parameters for drawing aircraft
            % scale size
            size = 2;
            fuse_l1    = 7;
            fuse_l2    = 4;
            fuse_l3    = 15;
            fuse_w     = 2;
            wing_l     = 6;
            wing_w     = 20;
            tail_l     = 3;
            tail_h     = 3;
            tailwing_w = 10;
            tailwing_l = 3;
            % colors
            red     = [1, 0, 0];
            green   = [0, 1, 0];
            blue    = [0, 0, 1];
            yellow  = [1,1,0];
            magenta = [0, 1, 1];
  

            % define vertices for aircraft
            V = [...
                 fuse_l1,             0,             0;...        % point 1
                 fuse_l2,            -fuse_w/2,     -fuse_w/2;... % point 2     
                 fuse_l2,             fuse_w/2,     -fuse_w/2;... % point 3     
                 fuse_l2,             fuse_w/2,      fuse_w/2;... % point 4
                 fuse_l2,            -fuse_w/2,      fuse_w/2;... % point 5
                -fuse_l3,             0,             0;...        % point 6
                 0,                   wing_w/2,      0;...        % point 7
                -wing_l,              wing_w/2,      0;...        % point 8
                -wing_l,             -wing_w/2,      0;...        % point 9
                 0,                  -wing_w/2,      0;...        % point 10
                -fuse_l3+tailwing_l,  tailwing_w/2,  0;...        % point 11
                -fuse_l3,             tailwing_w/2,  0;...        % point 12
                -fuse_l3,            -tailwing_w/2,  0;...        % point 13
                -fuse_l3+tailwing_l, -tailwing_w/2,  0;...        % point 14
                -fuse_l3+tailwing_l,  0,             0;...        % point 15
                -fuse_l3+tailwing_l,  0,             -tail_h;...  % point 16
                -fuse_l3,             0,             -tail_h;...  % point 17
                ]';
            V = size*V;   % rescale vertices
            
            % define faces for aircraft
            F = [...
                    1,  2,  3,  1;... % nose-top
                    1,  3,  4,  1;... % nose-left
                    1,  4,  5,  1;... % nose-bottom
                    1,  5,  2,  1;... % nose-right
                    2,  3,  6,  2;... % fuselage-top
                    3,  6,  4,  3;... % fuselage-left
                    4,  6,  5,  4;... % fuselage-bottom
                    2,  5,  6,  2;... % fuselage-right
                    7,  8,  9, 10;... % wing
                    11, 12, 13, 14;... % tailwing
                    6, 15, 17, 17;... % tail        
                ];  
            % define colors for aircraft
            colors = [...
                        yellow;... % nose-top
                        yellow;... % nose-left
                        yellow;... % nose-bottom
                        yellow;... % nose-right
                        blue;... % fuselage-top
                        blue;... % fuselage-left
                        red;... % fuselage-bottom
                        blue;... % fuselage-right
                        green;... % wing
                        green;... % tailwing
                        blue;... % tail
                    ];
        end
        %---------------------------
        function self = drawPath(self, path, S)
            if isequal(path.type, 'line')
                XX = [path.line_origin(1), path.line_origin(1)+S*path.line_direction(1)];
                YY = [path.line_origin(2), path.line_origin(2)+S*path.line_direction(2)];
                ZZ = [path.line_origin(3), path.line_origin(3)+S*path.line_direction(3)];
            elseif isequal(path.type, 'orbit')
                N = 100;
                th = [0:2*pi/N:2*pi];
                XX = path.orbit_center(1) + path.orbit_radius*cos(th);
                YY = path.orbit_center(2) + path.orbit_radius*sin(th);
                ZZ = path.orbit_center(3)*ones(size(th));
            end
    
            if isempty(self.path_handle)
                self.path_handle = plot3(YY,XX,-ZZ,'r');
            else
                set(self.path_handle,'XData', YY, 'YData', XX, 'ZData', -ZZ);
                drawnow
            end
        end 
        %---------------------------
        function self = drawWaypoints(self, waypoints, radius)
            if isequal(waypoints.type, 'straight_line')...
                    || isequal(waypoints.type, 'fillet')
                XX = [waypoints.ned(1,:)];
                YY = [waypoints.ned(2,:)];
                ZZ = [waypoints.ned(3,:)];
            elseif isequal(waypoints.type, 'dubins')
                XX = [];
                YY = [];
                for i=2:waypoints.num_waypoints
                    self.dubins_path.update(...
                        waypoints.ned(:,i-1),...
                        waypoints.course(i-1),...
                        waypoints.ned(:,i),...
                        waypoints.course(i),...
                        radius...
                        );
                    [tmpX, tmpY] = self.pointsAlongDubinsPath(0.1);
                    XX = [XX; tmpX];
                    YY = [YY; tmpY];     
                end
                ZZ = waypoints.ned(3,i)*ones(size(XX));
            end
    
            if isempty(self.waypoint_handle)
                self.waypoint_handle = plot3(YY,XX,-ZZ,'b', 'LineWidth', 2);
            else
                set(self.waypoint_handle,'XData', YY, 'YData', XX, 'ZData', -ZZ);
                drawnow
            end
        end 
        %---------------------------
        function self = drawNonSmoothedWaypoints(self, waypoints, radius)
            if isequal(waypoints.type, 'straight_line')...
                    || isequal(waypoints.type, 'fillet')
                XX = [waypoints.ned(1,:)];
                YY = [waypoints.ned(2,:)];
                ZZ = [waypoints.ned(3,:)];
            elseif isequal(waypoints.type, 'dubins')
                XX = [];
                YY = [];
                for i=2:waypoints.num_waypoints
                    self.dubins_path.update(...
                        waypoints.ned(:,i-1),...
                        waypoints.course(i-1),...
                        waypoints.ned(:,i),...
                        waypoints.course(i),...
                        radius...
                        );
                    [tmpX, tmpY] = self.pointsAlongDubinsPath(0.1);
                    XX = [XX; tmpX];
                    YY = [YY; tmpY];     
                end
                ZZ = waypoints.ned(3,i)*ones(size(XX));
            end
            plot3(YY,XX,-ZZ,'r', 'LineWidth', 2);
        end 
        %---------------------------
        function self = drawTree(self, tree, radius)
            if isequal(tree.type, 'straight_line')...
                    || isequal(tree.type, 'fillet')
                for i = 2:tree.num_waypoints
                    parent = tree.parent(i);
                    XX = [tree.ned(1,i), tree.ned(1,parent)];
                    YY = [tree.ned(2,i), tree.ned(2,parent)];
                    ZZ = [tree.ned(3,i), tree.ned(3,parent)];
                    plot3(YY, XX, -ZZ, 'm')
                end
            elseif isequal(tree.type, 'dubins')
                for i=2:tree.num_waypoints
                    parent = tree.parent(i);
                    self.dubins_path.update(...
                        tree.ned(:,parent),...
                        tree.course(parent),...
                        tree.ned(:,i),...
                        tree.course(i),...
                        radius...
                        );
                    [XX, YY] = self.pointsAlongDubinsPath(0.1);   
                    ZZ = tree.ned(3,i)*ones(size(XX));
                    plot3(YY, XX, -ZZ,'c')
                end
            end
        end 
        %---------------------------
        function [X, Y] = pointsAlongDubinsPath(self, Del)
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
            X = [];
            Y = [];
            for i=1:length(th)
                X = [X; self.dubins_path.center_s(1)+self.dubins_path.radius*cos(th(i))]; 
                Y = [Y; self.dubins_path.center_s(2)+self.dubins_path.radius*sin(th(i))];
            end
  
            % points along straight line 
            sig = 0;
            while sig<=1
                X = [X; (1-sig)*self.dubins_path.r1(1) + sig*self.dubins_path.r2(1)];
                Y = [Y; (1-sig)*self.dubins_path.r1(2) + sig*self.dubins_path.r2(2)];
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
                X = [X; self.dubins_path.center_e(1)+self.dubins_path.radius*cos(th(i))]; 
                Y = [Y; self.dubins_path.center_e(2)+self.dubins_path.radius*sin(th(i))];
            end
        end
        %---------------------------
        function drawMap(self, map)
            % draw map of the world:  buildings
            V = [];
            F = [];
            patchcolors = [];
            count = 0;
            for i=1:map.num_city_blocks
                for j=1:map.num_city_blocks
                    [Vtemp,Ftemp,patchcolorstemp] = self.buildingVertFace(...
                      map.building_north(i),...
                      map.building_east(j),...
                      map.building_width,...
                      map.building_height(i, j));
                    V = [V; Vtemp];
                    Ftemp = Ftemp + count;
                    F = [F; Ftemp];
                    count = count + 8;
                    patchcolors = [patchcolors;patchcolorstemp];
                end
            end
            patch('Vertices', V,...
                  'Faces', F,...
                  'FaceVertexCData', patchcolors,...
                  'FaceColor','flat');
        end
        %---------------------------
        function [V,F,patchcolors] = buildingVertFace(self, n, e, width, height)
            %   define patches for a building located at (x,y)
            % vertices of the building
            V = [...
                e+width/2, n+width/2, 0;...
                e+width/2, n-width/2, 0;...
                e-width/2, n-width/2, 0;...
                e-width/2, n+width/2, 0;...
                e+width/2, n+width/2, height;...
                e+width/2, n-width/2, height;...
                e-width/2, n-width/2, height;...
                e-width/2, n+width/2, height;...
                ];    
        % define faces of fuselage
        F = [...
            1, 4, 8, 5;... % North Side
            1, 2, 6, 5;... % East Side
            2, 3, 7, 6;... % South Side
            3, 4, 8, 7;... % West Side
            5, 6, 7, 8;... % Top
            ];   
        mygreen = [0, 1, 0];
        myyellow = [1,1,0];
        patchcolors = [...
            mygreen;... % North
            mygreen;... % East
            mygreen;... % South
            mygreen;... % West
            myyellow;...  % Top
            ];
        end
    end
end