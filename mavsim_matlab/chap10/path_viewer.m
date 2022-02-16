% path viewer for mavsim_matlab
%
% mavsim_matlab 
%     - Beard & McLain, PUP, 2012
%     - Last updated:  
%         3/12/2019 - RWB
classdef path_viewer < handle
    properties
        aircraft_handle
    	Vertices
    	Faces
    	facecolors
        plot_initialized
    end
    %--------------------------------
    methods
        %------constructor-----------
        function self = path_viewer
            self.aircraft_handle = [];
            [self.Vertices, self.Faces, self.facecolors] = self.define_mav();
            self.plot_initialized = 0;           
        end
        %---------------------------
        function self=update(self, path, state)
            %state = [pn, pe, h, Va, alpha, beta, phi, theta, chi, p, q, r, Vg, wn, we, psi, gyro_bx, gyro_by, gyro_bz]
            
            if self.plot_initialized==0
                S = 1000;
                figure(1); clf;
                if isequal(path.flag, 'line')
                    XX = [path.line_origin(1), path.line_origin(1)+S*path.line_direction(1)];
                    YY = [path.line_origin(2), path.line_origin(2)+S*path.line_direction(2)];
                    ZZ = [path.line_origin(3), path.line_origin(3)+S*path.line_direction(3)];
                else
                    N = 100;
                    th = [0:2*pi/N:2*pi];
                    XX = path.orbit_center(1) + path.orbit_radius*cos(th);
                    YY = path.orbit_center(2) + path.orbit_radius*sin(th);
                    ZZ = path.orbit_center(3)*ones(size(th));
                end
                plot3(YY,XX,-ZZ,'r')
                hold on                
                self=self.drawBody(state.pn, state.pe, -state.h,...
                                   state.phi, state.theta, state.psi);
                title('UAV')
                xlabel('East')
                ylabel('North')
                zlabel('-Down')
                view(0, 90)  % set the view angle for figure
                axis([-S, S, -S, S, -.1, S]);
                axis square;
                grid on
                self.plot_initialized = 1;
            else
                self=self.drawBody(state.pn, state.pe, -state.h,... 
                                   state.phi, state.theta, state.psi);
                %CameraPosition = ([state.pn; state.pe; state.h]);
            end
        end
        %---------------------------
        function self = drawBody(self, pn, pe, pd, phi, theta, psi)
            Vertices = self.rotate(self.Vertices, phi, theta, psi);   % rotate rigid body  
            Vertices = self.translate(Vertices, pn, pe, pd);     % translate after rotation
            % transform vertices from NED to XYZ (for matlab rendering)
            R = [...
                0, 1, 0;...
                1, 0, 0;...
                0, 0, -1;...
                ];
            Vertices = R*Vertices;
            if isempty(self.aircraft_handle)
                self.aircraft_handle = patch('Vertices', Vertices', 'Faces', self.Faces,...
                                             'FaceVertexCData',self.facecolors,...
                                             'FaceColor','flat');
            else
                set(self.aircraft_handle,'Vertices',Vertices','Faces',self.Faces);
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
    end
end