%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% mavSimMatlab 
%     - Chapter 3 assignment for Beard & McLain, PUP, 2012
%     - Update history:  
%         12/18/2018 - RWB
%         1/15/2019 - RWB
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
run('../parameters/simulation_parameters')  % load SIM: simulation parameters
run('../parameters/aerosonde_parameters')  % load MAV: aircraft parameters

% initialize the mav viewer
addpath('../chap2'); mav_view = mav_viewer();  
addpath('../chap2'); data_view = data_viewer();

% initialize the video writer
VIDEO = 0;  % 1 means write video, 0 means don't write video
if VIDEO==1, video=video_writer('chap3_video.avi', SIM.ts_video); end


% initialize elements of the architecture
mav = mav_dynamics(SIM.ts_simulation, MAV);

% initialize the simulation time
sim_time = SIM.start_time;

% main simulation loop
disp('Type CTRL-C to exit');
while sim_time < SIM.end_time
    %-------vary forces and moments to check dynamics-------------
    fx = 10;
    fy = 0; % 10;
    fz = 0; % 10;
    Mx = 0; % 0.1;
    My = 0; % 0.1;
    Mz = 0; % 0.1;
    forces_moments = [fx; fy; fz; Mx; My; Mz];

    %-------physical system-------------
    mav.update(forces_moments, MAV);
    
    %-------update viewer-------------
    mav_view.update(mav.true_state);  % plot body of MAV
    data_view.update(mav.true_state,... % true states
                     mav.true_state,... % estimated states
                     mav.true_state,... % commmanded states
                     SIM.ts_simulation); 
    if VIDEO, video.update(sim_time);  end

    %-------increment time-------------
    sim_time = sim_time + SIM.ts_simulation;
end

if VIDEO, video.close(); end

