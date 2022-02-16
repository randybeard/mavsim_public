% size of the waypoint array used for the path planner.  This is the
% maximum number of waypoints that might be transmitted to the path
% manager.
PLAN.size_waypoint_array = 100;

% airspeed commanded by planner
%PLAN.Va0 = MAV.u0;
PLAN.Va0 = 25;

% max possible roll angle
PLAN.phi_max = 20 * (pi/180);  

% minimum turn radius
PLAN.R_min = PLAN.Va0^2/MAV.gravity/tan(PLAN.phi_max);

% % create random city map
% city_width      = 2000;  % the city is of size (width)x(width)
% building_height = 300;   % maximum height of buildings
% %building_height = 1;   % maximum height of buildings (for camera)
% num_blocks      = 5;    % number of blocks in city
% street_width    = .8;   % percent of block that is street.
% P.map = createWorld(city_width, building_height, num_blocks, street_width);



