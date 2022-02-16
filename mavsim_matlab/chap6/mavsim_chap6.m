%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% mavsim_matlab 
%     - Chapter 6 assignment for Beard & McLain, PUP, 2012
%     - Update history:  
%         2/12/2019 - RWB
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
run('../parameters/simulation_parameters')  % load SIM: simulation parameters
run('../parameters/aerosonde_parameters')  % load MAV: aircraft parameters

% initialize the mav viewer
addpath('../chap2'); mav_view = mav_viewer();  
addpath('../chap3'); data_view = data_viewer();

% initialize the video writer
VIDEO = 0;  % 1 means write video, 0 means don't write video
if VIDEO==1, video=video_writer('chap6_video.avi', SIM.ts_video); end

% initialize elements of the architecture
addpath('../chap4'); wind = wind_simulation(SIM.ts_simulation);
addpath('../chap4'); mav = mav_dynamics(SIM.ts_simulation, MAV);
addpath('../chap6'); ctrl = autopilot(SIM.ts_simulation);

addpath('../message_types'); commands = msg_autopilot();
addpath('../tools');

% arguments to signals are amplitude, frequency, start_time, dc_offset
Va_command = signals(3, 0.01, 2, 25);
h_command = signals(10, 0.02, 0, 100);
chi_command = signals(45*pi/180, 0.015, 5, 180*pi/180);

% initialize the simulation time
sim_time = SIM.start_time;

% main simulation loop
disp('Type CTRL-C to exit');
while sim_time < SIM.end_time
    %-------controller-------------
    estimated_state = mav.true_state;  % uses true states in the control
    commands.airspeed_command = Va_command.square(sim_time);
    commands.course_command = chi_command.square(sim_time);
    commands.altitude_command = h_command.square(sim_time);
    [delta, commanded_state] = ctrl.update(commands, estimated_state);
    
    %-------physical system-------------
    current_wind = wind.update();
    mav.update(delta, current_wind, MAV);
    
    %-------update viewer-------------
    mav_view.update(mav.true_state);       % plot body of MAV
    data_view.update(mav.true_state,...    % true states
                     estimated_state,...   % estimated states
                     commanded_state,...   % commmanded states
                     SIM.ts_simulation); 
    if VIDEO==1, video.update(sim_time);  end

    %-------increment time-------------
    sim_time = sim_time + SIM.ts_simulation;
end

if VIDEO==1, video.close(); end

