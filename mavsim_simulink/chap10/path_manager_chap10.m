% path manager
%
% Modified:  
%   - 3/25/2010 - RWB
%
% output is:
%   flag - if flag==1, follow waypoint path
%          if flag==2, follow orbit
%   
%   Va^d - desired airspeed
%   r    - inertial position of start of waypoint path
%   q    - unit vector that defines inertial direction of waypoint path
%   c    - center of orbit
%   rho  - radius of orbit
%   lambda = direction of orbit (+1 for CW, -1 for CCW)
%
function out = path_manager_chap10(in)

  NN = 0;
  waypoints = in(1+NN);
  NN = NN + 1;
  pn        = in(1+NN);
  pe        = in(2+NN);
  h         = in(3+NN);
  Va        = in(4+NN);
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
  NN = NN + 16;
  t         = in(1+NN);
 

   if 1
    % define waypoint path
    flag   = 1;
    Va_d   = 35;
    %r      = [0; 0; -100];    
    r      = [-500; -1000; -100];   % devo sim setup
    %q      = [-1/2; -1; -0.05];
    q = [0.5; 1; 0];   % devo sim setup
    q      = q/norm(q);
    c      = [0;0;0];          % not used for waypoint path
    rho    = 0;          % not used for waypoint path
    lambda = 0;          % not used for waypoint path

   else
    % define orbit
    flag   = 2;
    Va_d   = 35;            % desired airspeed
    r      = [0; 0; 0];     % not used for orbit
    q      = [1; 0; 0];     % not used for orbit
    q      = q/norm(q);
    c      = [1;1;-100];  % center of orbit
    rho    = 400;            % radius of orbit
    lambda = 1;             % direction of orbit

   end
   
    out = [flag; Va_d; r; q; c; rho; lambda];

end