% path planner
%
% Modified:  
%   - 4/06/2010 - RWB
%   - 3/19/2019 - RWB
%
% output is a vector containing P.num_waypoints waypoints
%
% input is the map of the environment
function out = path_planner_chap11(in, PLAN)

  NN = 0;
  % pn        = in(1+NN);
  % pe        = in(2+NN);
  % h         = in(3+NN);
  % Va      = in(4+NN);
  % alpha   = in(5+NN);
  % beta    = in(6+NN);
  % phi     = in(7+NN);
  % theta   = in(8+NN);
  % chi     = in(9+NN);
  % p       = in(10+NN);
  % q       = in(11+NN);
  % r       = in(12+NN);
  % Vg      = in(13+NN);
  % wn      = in(14+NN);
  % we      = in(15+NN);
  % psi     = in(16+NN);
  % flag_new_waypoints =  in(17+NN);
  NN = NN + 17;
  % t         =  in(1+NN);


  num_waypoints = 4;
  % format for each point is [pn, pe, pd, chi, Va^d] where the position
  % of the waypoint is (pn, pe, pd), the desired course at the waypoint
  % is chi, and the desired airspeed between waypoints is Va
  % if chi!=-9999, then Dubins paths will be used between waypoints.
  if 1
    wpp = [...
            0,   0,   -100, -9999, PLAN.Va0;...
            1200, 0,   -100, -9999, PLAN.Va0;...
            0,   1200, -100, -9999, PLAN.Va0;...
            1200, 1200, -100, -9999, PLAN.Va0;...
           ];
  else  % Dubins
    wpp = [...
            0,    0,    -100,  0,           PLAN.Va0;...
            1200, 0,    -100,  45*pi/180,   PLAN.Va0;...
            0,    1200, -100,  45*pi/180,   PLAN.Va0;...
            1200, 1200, -100, -135*pi/180,  PLAN.Va0;...
           ];
  end      
  for i=5:PLAN.size_waypoint_array
      wpp = [...
          wpp;...
          -9999, -9999, -9999, -9999, -9999;...
          ];
  end
  
  out = [num_waypoints; reshape(wpp', 5*PLAN.size_waypoint_array, 1)]; 

end