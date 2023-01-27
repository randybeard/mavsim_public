"""
mavsimPy
    - Chapter 2 launch file for Beard & McLain, PUP, 2012
    - Update history:  
        12/27/2018 - RWB
        1/17/2019 - RWB
        1/5/2023 - DLC
"""
import sys
sys.path.append('../..')
import parameters.simulation_parameters as SIM
from viewers.spacecraft_viewer import SpaceCraftViewer
from message_types.msg_state import MsgState
from tools.quit_listener import QuitListener

quitter = QuitListener()

VIDEO = False
if VIDEO is True:
    from viewers.video_writer import VideoWriter
    video = VideoWriter(video_name="appc_video.avi",
                        bounding_box=(0, 0, 1000, 1000),
                        output_rate=SIM.ts_video)

# initialize the visualization
spacecraft_view = SpaceCraftViewer()  # initialize the mav viewer

# initialize elements of the architecture
state = MsgState()

# initialize the simulation time
sim_time = SIM.start_time
motions_time = 0
time_per_motion = 3
end_time = 20

# main simulation loop
while sim_time < end_time:
    # -------vary states to check viewer-------------
    if motions_time < time_per_motion:
        state.north += 10*SIM.ts_simulation
    elif motions_time < time_per_motion*2:
        state.east += 10*SIM.ts_simulation
    elif motions_time < time_per_motion*3:
        state.altitude += 10*SIM.ts_simulation
    elif motions_time < time_per_motion*4:
        state.psi += 0.1*SIM.ts_simulation
    elif motions_time < time_per_motion*5:
        state.theta += 0.1*SIM.ts_simulation
    else:
        state.phi += 0.1*SIM.ts_simulation
    # -------update viewer and video-------------
    spacecraft_view.update(state)
    spacecraft_view.process_app()

    # -------increment time-------------
    sim_time += SIM.ts_simulation
    motions_time += SIM.ts_simulation
    if motions_time >= time_per_motion*6:
        motions_time = 0

    # -------update video---------------
    if VIDEO is True:
        video.update(sim_time)

    # -------Check to Quit the Loop-------
    if quitter.check_quit():
        break

if VIDEO is True:
    video.update(sim_time)

print("Press Ctrl-Q to exit...")