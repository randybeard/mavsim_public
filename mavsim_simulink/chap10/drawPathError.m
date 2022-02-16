function drawPathError(uu)

    % process inputs to function
    pn       = uu(1);       % inertial North position     
    pe       = uu(2);       % inertial East position
    pd       = uu(3);       % inertial Down position
    u        = uu(4);       % body frame velocities
    v        = uu(5);       
    w        = uu(6);       
    phi      = uu(7);       % roll angle         
    theta    = uu(8);       % pitch angle     
    psi      = uu(9);       % yaw angle     
    p        = uu(10);      % roll rate
    q        = uu(11);      % pitch rate     
    r        = uu(12);      % yaw rate    
    t        = uu(13);      % time
    NN = 13;
    flag_path = uu(1+NN);      % path flag
    r_path    = [uu(3+NN); uu(4+NN); uu(5+NN)];
    q_path    = [uu(6+NN); uu(7+NN); uu(8+NN)];
    c_orbit   = [uu(9+NN); uu(10+NN); uu(11+NN)];
    rho_orbit = uu(12+NN);
    lam_orbit = uu(13+NN);


    % define persistent variables 
    persistent aircraft_handle;  % figure handle for MAV
    persistent Vertices
    persistent Faces
    persistent facecolors

    % first time function is called, initialize plot and persistent vars
    if t==0

        figure(4), clf
        %S = 4000;
        S = 2000;
        switch flag_path
            case 1
                XX = [r_path(1), r_path(1)+S*q_path(1)];
                YY = [r_path(2), r_path(2)+S*q_path(2)];
                ZZ = [r_path(3), r_path(3)+S*q_path(3)];
            case 2
                N = 100;
                th = [0:2*pi/N:2*pi];
                XX = c_orbit(1) + rho_orbit*cos(th);
                YY = c_orbit(2) + rho_orbit*sin(th);
                ZZ = c_orbit(3)*ones(size(th));
        end
        plot3(YY,XX,-ZZ,'r')
        hold on
        [Vertices,Faces,facecolors] = defineAircraftBody;                
        aircraft_handle = drawBody(Vertices,Faces,facecolors,...
                                   pn,pe,pd,phi,theta,psi,...
                                   [], 'normal');
        title('UAV')
        xlabel('East')
        ylabel('North')
        zlabel('-Down')
        view(0,90)  % set the view angle for figure
        axis([-S,S,-S,S,-.1,  S]);
        axis square;
        grid on
        
        
    % at every other time step, redraw MAV
    else 
        drawBody(Vertices,Faces,facecolors,...
                     pn,pe,pd,phi,theta,psi,...
                     aircraft_handle);
    end
%    figure(1), plot3(pe,pn,-pd,'.k');
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function handle = drawBody(V,F,colors,...
                               pn, pe, pd, phi, theta, psi,...
                               handle, mode)
  V = rotate(V', phi, theta, psi)';  % rotate rigid body  
  V = translate(V', pn, pe, pd)';  % translate after rotation

  % transform vertices from NED to XYZ (for matlab rendering)
  R = [...
      0, 1, 0;...
      1, 0, 0;...
      0, 0, -1;...
      ];
  V = V*R;

  if isempty(handle)
    handle = patch('Vertices', V, 'Faces', F,...
                 'FaceVertexCData',colors,...
                 'FaceColor','flat',...
                 'EraseMode', mode);
  else
    set(handle,'Vertices',V,'Faces',F);
    drawnow
  end
  
end 


%%%%%%%%%%%%%%%%%%%%%%%
function XYZ=rotate(XYZ,phi,theta,psi)
  % define rotation matrix
  R_roll = [...
          1, 0, 0;...
          0, cos(phi), -sin(phi);...
          0, sin(phi), cos(phi)];
  R_pitch = [...
          cos(theta), 0, sin(theta);...
          0, 1, 0;...
          -sin(theta), 0, cos(theta)];
  R_yaw = [...
          cos(psi), -sin(psi), 0;...
          sin(psi), cos(psi), 0;...
          0, 0, 1];

  % rotate vertices
  XYZ = R_yaw*R_pitch*R_roll*XYZ;
  
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% translate vertices by pn, pe, pd
function XYZ = translate(XYZ,pn,pe,pd)

  XYZ = XYZ + repmat([pn;pe;pd],1,size(XYZ,2));
  
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% define aircraft vertices and faces
function [V,F,colors] = defineAircraftBody

% parameters for drawing aircraft
  % scale size
  size = 4;
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
  

% define vertices and faces for aircraft
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
  ];
  
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

  V = size*V;   % rescale vertices

end
  