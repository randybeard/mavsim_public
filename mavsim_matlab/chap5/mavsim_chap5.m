%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% mavSimMatlab 
%     - Chapter 5 assignment for Beard & McLain, PUP, 2012
%     - Update history:  
%         2/5/2019 - RWB
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
run('../parameters/simulation_parameters')  % load SIM: simulation parameters
run('../parameters/aerosonde_parameters')  % load MAV: aircraft parameters

% initialize the mav viewer
addpath('../chap2'); mav_view = mav_viewer();  
addpath('../chap3'); data_view = data_viewer();

% initialize the video writer
VIDEO = 0;  % 1 means write video, 0 means don't write video
if VIDEO==1
    video=video_writer('chap5_video.avi', SIM.ts_video);
end

% initialize elements of the architecture
addpath('../chap4'); 
wind = wind_simulation(SIM.ts_simulation);
mav = mav_dynamics(SIM.ts_simulation, MAV);

% compute trim
addpath('../chap5');
Va = 25;
gamma = 0*pi/180;
[trim_state, trim_input] = compute_trim(mav, Va, gamma, MAV);
mav.state = trim_state;
delta = trim_input;

% compute linearized models
[A_lon, B_lon, A_lat, B_lat] = compute_ss_model(mav, trim_state, trim_input, MAV);

% initialize the simulation time
sim_time = SIM.start_time;

% main simulation loop
disp('Type CTRL-C to exit');
while sim_time < SIM.end_time

    %-------physical system-------------
    %current_wind = wind.update();
    current_wind = zeros(6,1);
    mav.update_state(delta, current_wind, MAV);
    
    %-------update viewer-------------
    mav_view.update(mav.true_state);  % plot body of MAV
    data_view.update(mav.true_state,...  % true states
                     mav.true_state,...  % estimated states
                     mav.true_state,...  % commmanded states
                     SIM.ts_simulation); 
    if VIDEO==1
        video.update(sim_time);  
    end

    %-------increment time-------------
    sim_time = sim_time + SIM.ts_simulation;
end

if VIDEO==1
    video.close(); 
end

