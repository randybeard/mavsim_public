%% planRRTDubins
%%  - create a path from a start node to an end node
%%    using the RRT algorithm, assuming a Dubin dynamic model.
%%  
%% Last Modified - 7/10/2009 - R. Beard

function path_out=planRRTDubins(wpp_start, wpp_end, R_min, map)
    
    % segment Length
    segmentLength = 2.5*R_min;  % must be larger than 2*R_min

    % desired down position is down position of end node
    pd = wpp_end(3);
    
    % specify start and end nodes from wpp_start and wpp_end
    start_node = [wpp_start(1), wpp_start(2), pd, wpp_start(4), 0, 0, 0];
    end_node = [wpp_end(1), wpp_end(2), pd, wpp_end(4), 0, 0, 0];
    % format is [N, E, D, chi, cost, parent, connect_to_goal_flag]
 
    % establish tree starting with the start node
    tree = start_node;
        
    % check to see if start_node connects directly to end_node
    if ( (norm(start_node(1:3)-end_node(1:3))<segmentLength )...
        &&(norm(start_node(1:3)-end_node(1:3))>=2*R_min)...
        &&(collision(start_node,end_node,map,R_min)==0) )
      path = [start_node; end_node];
    else
      numPaths = 0;
      while numPaths<3,
          [tree,flag] = extendTree(tree,end_node,segmentLength,map,R_min,pd);
          numPaths = numPaths + flag;
      end
    end

    % find path with minimum cost to end_node
    path = findMinimumPath(tree,end_node);
    path_out = smoothPath(path,map,R_min);
    plotmap(map,path,path_out,tree,R_min);
   
end
        
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% plotmap
%%   plot obstacles and path
function plotmap(map,path,smoothedPath,tree,R_min)
  
 % setup plot
  figure(3), clf
  axis([0,map.width,0,map.width,0,2*map.MaxHeight]);
  xlabel('E')
  ylabel('N')
  zlabel('h')
  hold on
  
  % plot buildings 
 
  V = [];
  F = [];
  patchcolors = [];
  count = 0;
  for i=1:map.NumBlocks
      for j=1:map.NumBlocks
        [Vtemp,Ftemp,patchcolorstemp] = buildingVertFace(map.buildings_n(i),...
            map.buildings_e(j),map.BuildingWidth,map.heights(j,i));
        V = [V; Vtemp];
        Ftemp = Ftemp + count;
        F = [F; Ftemp];
        count = count + 8;
        patchcolors = [patchcolors;patchcolorstemp];
      end
  end
  
  patch('Vertices', V, 'Faces', F,...
                 'FaceVertexCData',patchcolors,...
                 'FaceColor','flat');
 
  % draw tree
  for i=2:size(tree,1)
      X = [];
      Y = [];
      dubinspath = dubinsParameters(tree(tree(i,6),:),tree(i,:),R_min);
      [tmpX,tmpY] = pointsAlongDubinsPath(dubinspath,0.1);
      X = [X; tmpX];
      Y = [Y; tmpY];     
      Z = path(1,3)*ones(size(X));
      plot3(Y,X,-Z,'b')
  end
  
  % draw path
  X = [];
  Y = [];
  for i=2:size(path,1)
      dubinspath = dubinsParameters(path(i-1,:),path(i,:),R_min);
      [tmpX,tmpY] = pointsAlongDubinsPath(dubinspath,0.1);
      X = [X; tmpX];
      Y = [Y; tmpY];     
  end
  Z = path(1,3)*ones(size(X));
  plot3(Y,X,-Z,'r','linewidth',2);

  % draw smooth path
  X = [];
  Y = [];
  for i=2:size(smoothedPath,1)
      dubinspath = dubinsParameters(smoothedPath(i-1,:),smoothedPath(i,:),R_min);
      [tmpX,tmpY] = pointsAlongDubinsPath(dubinspath,0.1);
      X = [X; tmpX];
      Y = [Y; tmpY];     
  end
  Z = smoothedPath(1,3)*ones(size(X));
  plot3(Y,X,-Z,'k','linewidth',2);

end
  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% buildingVertFace(x,y,width,height)
%%   define patches for a building located at (x,y)
function [V,F,patchcolors] = buildingVertFace(n,e,width,height)
 
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

  myred = [1, 0, 0];
  mygreen = [0, 1, 0];
  myblue = [0, 0, 1];
  myyellow = [1,1,0];
  mymagenta   = [0, 1, 1];

  patchcolors = [...
    mygreen;... % North
    mygreen;... % East
    mygreen;... % South
    mygreen;... % West
    myyellow;...  % Top
    ];

end
  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% generateRandomNode
%%   create a random node (initialize)
function node=generateRandomNode(map,pd)
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% downAtNE
%%   find the world down coordinate at a specified (n,e) location
function down = downAtNE(map, n, e)
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% collision
%%   check to see if a node is in collsion with obstacles
function collision_flag = collision(start_node, end_node, map, R_min)
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% canEndConnectToTree
%%   check to see if the end node can connect to the tree
function flag = canEndConnectToTree(tree,end_node,minDist,map)
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% extendTree
%%   extend tree by randomly selecting point and growing tree toward that
%%   point
function [new_tree,flag] = extendTree(tree,end_node,segmentLength,map,R_min,pd)
end
  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% findMinimumPath
%%   find the lowest cost path to the end node
function path = findMinimumPath(tree,end_node)
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% smoothPath
%%   smooth the waypoint path 
function newPath = smoothPath(path,map,R_min)
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% rotz(theta)
%%   rotation matrix about the z axis.
function R = rotz(theta)
    R = [...
        cos(theta), -sin(theta), 0;...
        sin(theta), cos(theta), 0;...
        0, 0, 1;...
        ];
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

