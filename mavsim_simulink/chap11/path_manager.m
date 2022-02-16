% path manager
%
% Modified:  
%   - 3/25/2010 - RWB
%   - 4/7/2010 - RWB
%   - 3/19/2019 - RWB
%
% input is:
%   num_waypoints - number of waypoint configurations
%   waypoints    - an array of dimension 5 by P.size_waypoint_array.
%                - the first num_waypoints rows define waypoint
%                  configurations
%                - format for each waypoint configuration:
%                  [wn, we, wd, chi_d, Va_d]
%                  where the (wn, we, wd) is the NED position of the
%                  waypoint, chi_d is the desired course at the waypoint,
%                  and Va_d is the desired airspeed along the path.  If
%                  abs(chi_d)<2*pi then Dubins paths will be followed
%                  between waypoint configurations.  If abs(chi_d)>=2*pi
%                  then straight-line paths (with fillets) are commanded.
%
% output is:
%   flag - if flag==1, follow waypoint path
%          if flag==2, follow orbit 
%   Va_d - desired airspeed
%   r    - inertial position of start of waypoint path
%   q    - unit vector that defines inertial direction of waypoint path
%   c    - center of orbit
%   rho  - radius of orbit
%   lambda = direction of orbit (+1 for CW, -1 for CCW)
%
function out = path_manager(in,PLAN)

  persistent start_of_simulation
  
  t = in(end);
  if t==0
      start_of_simulation = 1;
  end

  NN = 0;
  num_waypoints = in(1+NN);
  if num_waypoints==0 % start of simulation
      flag   = 1;  % following straight line path
      Va_d   = PLAN.Va0; % desired airspeed along waypoint path
      NN = NN + 1 + 5*PLAN.size_waypoint_array;
      pn        = in(1+NN);
      pe        = in(2+NN);
      h         = in(3+NN);
      chi       = in(9+NN);
      r         = [pn; pe; -h];
      q         = [cos(chi); sin(chi); 0];
      c         = [0; 0; 0];
      rho       = 0;
      lambda    = 0;
      state     =  in(1+NN:16+NN);
      flag_need_new_waypoints = 1;
      out = [flag; Va_d; r; q; c; rho; lambda; state; flag_need_new_waypoints];
  else
    waypoints = reshape(in(2+NN:5*PLAN.size_waypoint_array+1+NN),5,PLAN.size_waypoint_array);
  
    if abs(waypoints(4,1))>=2*pi
        % follows straight-lines and switches at waypoints
        out = path_manager_line(in,PLAN,start_of_simulation);  
        % smooths through waypoints with fillets
        %out = path_manager_fillet(in,PLAN,start_of_simulation);  
        start_of_simulation=0;
    else
        % follows Dubins paths between waypoint configurations
        out = path_manager_dubins(in,PLAN,start_of_simulation); 
        start_of_simulation=0;
    end
  end
  

end