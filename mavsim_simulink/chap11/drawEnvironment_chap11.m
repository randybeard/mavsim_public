function drawEnvironment_chap11(uu,PLAN)

    % process inputs to function
    NN = 0;
    pn       = uu(1+NN);       % inertial North position     
    pe       = uu(2+NN);       % inertial East position
    pd       = uu(3+NN);       % inertial Down position
    u        = uu(4+NN);       % body frame velocities
    v        = uu(5+NN);       
    w        = uu(6+NN);       
    phi      = uu(7+NN);       % roll angle         
    theta    = uu(8+NN);       % pitch angle     
    psi      = uu(9+NN);       % yaw angle     
    p        = uu(10+NN);      % roll rate
    q        = uu(11+NN);      % pitch rate     
    r        = uu(12+NN);      % yaw rate    
    t        = uu(13+NN);      % time
    
    NN = NN + 13;
    path     = uu(1+NN:13+NN); 
    NN = NN + 13;
    num_waypoints = uu(1+NN);
    waypoints     = reshape(uu(2+NN:5*num_waypoints+1+NN),5,num_waypoints)'; 


    % define persistent variables 
    persistent aircraft_handle  % figure handle for MAV
    persistent path_handle      % handle for straight-line or orbit path
    persistent waypoint_handle  % handle for waypoints
    persistent Faces
    persistent Vertices
    persistent facecolors
    persistent old_path  % used to signal a path re-draw
    persistent old_waypoints % used to signal a waypoint re-draw

    S = 2000; % plot size
    
    % first time function is called, initialize plot and persistent vars
    if t==0

        figure(1), clf
        scale = 4;
       [Vertices,Faces,facecolors] = defineAircraftBody(scale);                              
        aircraft_handle = drawBody(Vertices,Faces,facecolors,...
                                   pn,pe,pd,phi,theta,psi,...
                                   []);
        hold on
        waypoint_handle = drawWaypoints(waypoints, PLAN.R_min, []);
        path_handle = drawPath(path, S, []);
        
        title('UAV')
        xlabel('East')
        ylabel('North')
        zlabel('-Down')
        axis([-S/5,S,-S/5,S,0,300]);
        view(-40,70)  % set the view angle for figure
        grid on
        
        old_waypoints = waypoints;
        old_path = path;
        
    % at every other time step, redraw MAV
    else 
        drawBody(Vertices,Faces,facecolors,...
                     pn,pe,pd,phi,theta,psi,...
                     aircraft_handle);
        if norm(waypoints-old_waypoints) > 0.1
            drawWaypoints(waypoints, PLAN.R_min, waypoint_handle);
            old_waypoints = waypoints;
        end
        if norm(path - old_path) > 0.1
            drawPath(path, S, path_handle);
            old_path = path;
        end
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function handle = drawBody(V,F,colors,...
                               pn, pe, pd, phi, theta, psi,...
                               handle)
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
                 'FaceColor','flat');
  else
    set(handle,'Vertices',V,'Faces',F);
    drawnow
  end
  
end 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function handle = drawPath(path, S, handle)
    flag = path(1); 
    r    = [path(3); path(4); path(5)];
    q    = [path(6); path(7); path(8)];
    c    = [path(9); path(10); path(11)];
    rho  = path(12);
    lam  = path(13);

    switch flag
        case 1
            XX = [r(1), r(1)+S*q(1)];
            YY = [r(2), r(2)+S*q(2)];
            ZZ = [r(3), r(3)+S*q(3)];
        case 2
            N = 100;
            th = [0:2*pi/N:2*pi];
            XX = c(1) + rho*cos(th);
            YY = c(2) + rho*sin(th);
            ZZ = c(3)*ones(size(th));
    end
    
    if isempty(handle)
        handle = plot3(YY,XX,-ZZ,'r');
    else
        set(handle,'XData', YY, 'YData', XX, 'ZData', -ZZ);
        drawnow
    end
end 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function handle = drawWaypoints(waypoints, R_min, handle)

    if waypoints(1,4)==-9999 % check to see if Dubins paths
        XX = [waypoints(:,1)];
        YY = [waypoints(:,2)];
        ZZ = [waypoints(:,3)];
    else
        XX = [];
        YY = [];
        for i=2:size(waypoints,1)
            dubinspath = dubinsParameters(waypoints(i-1,:),waypoints(i,:),R_min);
            [tmpX,tmpY] = pointsAlongDubinsPath(dubinspath,0.1);
            XX = [XX; tmpX];
            YY = [YY; tmpY];     
        end
        ZZ = waypoints(i,3)*ones(size(XX));
    end
    
    if isempty(handle)
        handle = plot3(YY,XX,-ZZ,'b');
    else
        set(handle,'XData', YY, 'YData', XX, 'ZData', -ZZ);
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


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% pointsAlongDubinsPath
%%   Find points along Dubin's path separted by Del (to be used in
%%   collision detection)
function [X,Y] = pointsAlongDubinsPath(dubinspath,Del)


  % points along start circle
  th1 = mod(atan2(dubinspath.ps(2)-dubinspath.cs(2),dubinspath.ps(1)-dubinspath.cs(1)),2*pi);
  th2 = mod(atan2(dubinspath.w1(2)-dubinspath.cs(2),dubinspath.w1(1)-dubinspath.cs(1)),2*pi);
  if dubinspath.lams>0
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
    X = [X; dubinspath.cs(1)+dubinspath.R*cos(th(i))]; 
    Y = [Y; dubinspath.cs(2)+dubinspath.R*sin(th(i))];
  end
  
  % points along straight line 
  sig = 0;
  while sig<=1
      X = [X; (1-sig)*dubinspath.w1(1) + sig*dubinspath.w2(1)];
      Y = [Y; (1-sig)*dubinspath.w1(2) + sig*dubinspath.w2(2)];
      sig = sig + Del;
  end
    
  % points along end circle
  th2 = mod(atan2(dubinspath.pe(2)-dubinspath.ce(2),dubinspath.pe(1)-dubinspath.ce(1)),2*pi);
  th1 = mod(atan2(dubinspath.w2(2)-dubinspath.ce(2),dubinspath.w2(1)-dubinspath.ce(1)),2*pi);
  if dubinspath.lame>0
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
    X = [X; dubinspath.ce(1)+dubinspath.R*cos(th(i))]; 
    Y = [Y; dubinspath.ce(2)+dubinspath.R*sin(th(i))];
  end
  
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% define aircraft vertices and faces
function [V,F,colors] = defineAircraftBody(scale)


% parameters for drawing aircraft
  % scale size
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

  V = scale*V;   % rescale vertices

end
  