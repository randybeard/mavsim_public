%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% mavSimMatlab 
%     - Chapter 2 assignment for Beard & McLain, PUP, 2012
%     - Update history:  
%         12/15/2018 - RWB
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

run('../parameters/simulation_parameters')  % load SIM: simulation parameters

% initialize messages
addpath('../message_types'); state = msg_state();  

% initialize the mav viewer
%addpath('../chap2'); mav_view = mav_viewer();
addpath('../chap2'); mav_view = spacecraft_viewer();

% initialize the video writer
VIDEO = 0;  % 1 means write video, 0 means don't write video
if VIDEO==1, video=video_writer('chap2_video.avi', SIM.ts_video); end

% initialize the simulation time
sim_time = SIM.start_time;

% main simulation loop
disp('Type CTRL-C to exit');
while sim_time < SIM.end_time
    %-------vary states to check viewer-------------
    if sim_time < SIM.end_time/6
        state.pn = state.pn + SIM.ts_simulation;
    elseif sim_time < 2*SIM.end_time/6
        state.pe = state.pe + SIM.ts_simulation;
    elseif sim_time < 3*SIM.end_time/6
        state.h = state.h + SIM.ts_simulation;
    elseif sim_time < 4*SIM.end_time/6
        state.phi = state.phi + 0.1*SIM.ts_simulation;
    elseif sim_time < 5*SIM.end_time/6
        state.theta = state.theta + 0.1*SIM.ts_simulation;
    else 
        state.psi = state.psi + 0.1*SIM.ts_simulation;
    end

    %-------update viewer-------------
    mav_view.update(state);
    if VIDEO, video.update(sim_time);  end

    %-------increment time-------------
    sim_time = sim_time + SIM.ts_simulation;
end

if VIDEO, video.close(); end
