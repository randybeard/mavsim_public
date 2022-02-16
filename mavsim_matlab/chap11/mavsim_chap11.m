%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% mavsim_matlab 
%     - Chapter 11 assignment for Beard & McLain, PUP, 2012
%     - Update history:  
%         3/26/2019 - RWB
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all, path(pathdef)  % clear all variables and reset path 
run('../parameters/simulation_parameters')  % load SIM
run('../parameters/aerosonde_parameters')  % load MAV
run('../parameters/control_parameters')  % load CTRL
run('../parameters/sensor_parameters')  % load SENSOR
run('../parameters/planner_parameters')  % load PLAN

% initialize the viewer
addpath('../chap11'); waypoint_view = waypoint_viewer();
addpath('../chap3'); data_view = data_viewer();

% initialize the video writer
VIDEO = 0;  % 1 means write video, 0 means don't write video
if VIDEO==1
    addpath('../chap2'); video=video_writer('chap11_video.avi', SIM.ts_video); 
end

% initialize elements of the architecture
addpath('../chap7');  mav = mav_dynamics(SIM.ts_simulation, MAV);
addpath('../chap4');  wind = wind_simulation(SIM.ts_simulation);
addpath('../chap6');  ctrl = autopilot(SIM.ts_simulation);
addpath('../chap8');  obsv = observer(SIM.ts_simulation);
addpath('../chap10'); path_follow = path_follower();
addpath('../chap11'); path_manage = path_manager();


% waypoint definition
addpath('../message_types'); 
waypoints = msg_waypoints();
waypoints.type = 'straight_line';
%waypoints.type = 'fillet';
%waypoints.type = 'dubins';
waypoints.num_waypoints = 4;
Va = PLAN.Va0;
waypoints.ned(:, 1:waypoints.num_waypoints) = ...
    [0,   0,   -100;...
     1000, 0,   -100;...
     0,   1000, -100;...
     1000, 1000, -100]';
waypoints.airspeed(1:waypoints.num_waypoints) = ...
    [Va,...
     Va,...
     Va,...
     Va];
waypoints.course = ...
    [0,...
     45*pi/180,...
     45*pi/180,...
     -135*pi/180];

% initialize the simulation time
sim_time = SIM.start_time;

% main simulation loop
disp('Type CTRL-C to exit');
while sim_time < SIM.end_time
    
    %-------observer-------------
    mav.update_sensors(MAV, SENSOR); % get sensor measurement
    measurements = mav.sensors;
    estimated_state = obsv.update(measurements, MAV);

    %-------path manager-----
    path = path_manage.update(waypoints, PLAN.R_min, estimated_state);
    waypoints.flag_waypoints_changed = 0;  % only draw waypoints once    
    
    %-------path follower-----
    autopilot_commands = path_follow.update(path, estimated_state);

    %-------controller-------------
    [delta, commanded_state] = ctrl.update(autopilot_commands, estimated_state);
    
    %-------physical system-------------
    current_wind = wind.update();
    mav.update(delta, current_wind, MAV);
    
    %-------update viewer-------------
    waypoint_view.update(waypoints, path, mav.true_state);
%     data_view.update(mav.true_state,...
%                      estimated_state,...
%                      commanded_state,...
%                      SIM.ts_simulation); 
    if VIDEO==1, video.update(sim_time);  end

    %-------increment time-------------
    sim_time = sim_time + SIM.ts_simulation;
end

if VIDEO==1, video.close(); end

