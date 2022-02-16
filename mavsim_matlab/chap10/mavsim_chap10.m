%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% mavsim_matlab 
%     - Chapter 10 assignment for Beard & McLain, PUP, 2012
%     - Update history:  
%         3/12/2019 - RWB
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all, path(pathdef)  % clear all variables and reset path 
run('../parameters/simulation_parameters')  % load SIM: simulation parameters
run('../parameters/aerosonde_parameters')  % load MAV: aircraft parameters
run('../parameters/control_parameters')  % load CTRL: control parameters
run('../parameters/sensor_parameters')  % load SENSOR: sensor parameters

% initialize the viewer
addpath('../chap10'); path_view = path_viewer();
addpath('../chap3'); data_view = data_viewer(); % use for debugging

% initialize the video writer
VIDEO = 0;  % 1 means write video, 0 means don't write video
if VIDEO==1, video=video_writer('chap10_video.avi', SIM.ts_video); end

% initialize elements of the architecture
addpath('../chap7');  mav = mav_dynamics(SIM.ts_simulation, MAV);
addpath('../chap4');  wind = wind_simulation(SIM.ts_simulation);
addpath('../chap6');  ctrl = autopilot(SIM.ts_simulation);
addpath('../chap8');  obsv = observer(SIM.ts_simulation);
addpath('../chap10'); path_follow = path_follower();

% path definition
addpath('../message_types'); path = msg_path();
%path.flag = 'line';
path.flag = 'orbit';
if isequal(path.flag, 'line')
    path.line_origin = [0.0; 0.0; -100.0];
    path.line_direction = [0.5; 1.0; 0.0];
    path.line_direction = path.line_direction / norm(path.line_direction);
else  % path.flag == 'orbit'
    path.orbit_center = [0.0; 0.0; -100.0];  
    path.orbit_radius = 200.0;  
    path.orbit_direction = 'CW';  
end

% initialize the simulation time
sim_time = SIM.start_time;

% main simulation loop
disp('Type CTRL-C to exit');
while sim_time < SIM.end_time
    
    %-------observer-------------
    mav.update_sensors(MAV, SENSOR); % get sensor measurement
    measurements = mav.sensors;
    estimated_state = obsv.update(measurements, MAV);

    %-------path follower-----
    autopilot_commands = path_follow.update(path, estimated_state);

    %-------controller-------------
    [delta, commanded_state] = ctrl.update(autopilot_commands, estimated_state);
    % use true states for debugging
    %[delta, commanded_state] = ctrl.update(autopilot_commands, mav.true_state);
    
    %-------physical system-------------
    current_wind = wind.update();
    mav.update(delta, current_wind, MAV);
    
    %-------update viewer-------------
    path_view.update(path, mav.true_state);
    % use for debugging
%     data_view.update(mav.true_state,...
%                      estimated_state,...
%                      commanded_state,...
%                      SIM.ts_simulation); 
    if VIDEO==1, video.update(sim_time);  end

    %-------increment time-------------
    sim_time = sim_time + SIM.ts_simulation;
end

if VIDEO==1, video.close(); end

