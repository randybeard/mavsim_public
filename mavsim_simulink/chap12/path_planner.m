% path planner
%
% Modified:  
%   - 4/06/2010 - RWB
%
% output is a vector containing P.num_waypoints waypoints
%
% input is the map of the environment and desired goal location
function out = path_planner(in, PLAN, map, SIM)

 NN = 0;
  pn        = in(1+NN);
  pe        = in(2+NN);
  h         = in(3+NN);
  % Va      = in(4+NN);
  % alpha   = in(5+NN);
  % beta    = in(6+NN);
  % phi     = in(7+NN);
  % theta   = in(8+NN);
  chi     = in(9+NN);
  % p       = in(10+NN);
  % q       = in(11+NN);
  % r       = in(12+NN);
  % Vg      = in(13+NN);
  % wn      = in(14+NN);
  % we      = in(15+NN);
  % psi     = in(16+NN);
  flag_new_waypoints =  in(17+NN);
  NN = NN + 17;
  t         =  in(1+NN);
  
  % HACK:  This is a hack because the state is set right the first loop
  % through the simulation
  h = 100;
  % END HACK
    
  persistent num_waypoints
  persistent wpp
  
  if (t==0) || flag_new_waypoints
    % format for each point is [pn, pe, pd, chi, Va^d] where the position
    % of the waypoint is (pn, pe, pd), the desired course at the waypoint
    % is chi, and the desired airspeed between waypoints is Va
    % if chi!=-9999, then Dubins paths will be used between waypoints.
    switch 4
          case 1
            num_waypoints = 4;
            wpp = [...
                   0, 0, -100, -9999, PLAN.Va0;...
                    300, 0, -100, -9999, PLAN.Va0;...
                    0, 300, -100, -9999, PLAN.Va0;...
                    300, 300, -100, -9999, PLAN.Va0;...
                  ];
          case 2  % Dubins
            num_waypoints = 4;
            wpp = [...
                    0, 0, -100, 0, PLAN.Va0;...
                    300, 0, -100, 45*pi/180, PLAN.Va0;...
                    0, 300, -100, 45*pi/180, PLAN.Va0;...
                    300, 300, -100, -135*pi/180, PLAN.Va0;...
                  ];
          case 3  % path through city using straight-line RRT
               % current configuration
              wpp_start = [pn, pe, -h, chi, PLAN.Va0];
              % desired end waypoint
              if norm([pn; pe; -h]-[map.width; map.width; -h])<map.width/2
                  wpp_end = [0, 0, -h, chi, PLAN.Va0];
              else
                  wpp_end = [map.width, map.width, -h, chi, PLAN.Va0];
              end
              waypoints = planRRT(wpp_start, wpp_end, map);
              num_waypoints = size(waypoints,1);
              wpp = [];
              for i=1:num_waypoints
                  wpp = [...
                            wpp;...
                            waypoints(i,1), waypoints(i,2), waypoints(i,3), waypoints(i,4), PLAN.Va0;...
                        ];
              end
          case 4  % path through city using Dubin's paths
               % current configuration
              wpp_start = [pn, pe, -h, chi, PLAN.Va0];
              % desired end waypoint
              if norm([pn; pe; -h]-[map.width; map.width; -h])<map.width/2
                  wpp_end = [0, 0, -h, 0, PLAN.Va0];
              else
                  wpp_end = [map.width, map.width, -h, 0, PLAN.Va0];
              end
              waypoints = planRRTDubins(wpp_start, wpp_end, PLAN.R_min, map);
              num_waypoints = size(waypoints,1);
              wpp = [];
              for i=1:num_waypoints
                  wpp = [...
                            wpp;...
                            waypoints(i,1), waypoints(i,2), waypoints(i,3), waypoints(i,4), PLAN.Va0;...
                        ];
              end
    end
    for i=num_waypoints+1:PLAN.size_waypoint_array
        wpp = [...
                  wpp;...
                  -9999, -9999, -9999, -9999, -9999;...
               ];
    end  
  end
  
  out = [num_waypoints; reshape(wpp', 5*PLAN.size_waypoint_array, 1)];

end