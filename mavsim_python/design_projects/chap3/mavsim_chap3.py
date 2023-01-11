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
import numpy as np
import parameters.simulation_parameters as SIM
from viewers.mav_viewer import MavViewer
from viewers.data_viewer import DataViewer
from message_types.msg_delta import MsgDelta
from models.mav_dynamics_forces import MavDynamics
VIDEO = False
if VIDEO is True:
    from viewers.video_writer import VideoWriter
    video = VideoWriter(video_name="chap3_video.avi",
                        bounding_box=(0, 0, 1000, 1000),
                        output_rate=SIM.ts_video)

#initialize the viewers
mav_view = MavViewer()  # initialize the mav viewer
data_view = DataViewer()  # initialize view of data plots

# initialize elements of the architecture
mav = MavDynamics(SIM.ts_simulation)
delta = MsgDelta()

# initialize the simulation time
sim_time = SIM.start_time

# main simulation loop
print("Press Command-Q to exit...")
while sim_time < SIM.end_time:
    # -------vary forces and moments to check dynamics-------------
    fx = 0  # 10
    fy = 0  # 10
    fz = 100  # 10
    Mx = 0.1  # 0.1
    My = 0  # 0.1
    Mz = 0  # 0.1
    forces_moments = np.array([[fx, fy, fz, Mx, My, Mz]]).T

    # -------physical system-------------
    mav.update(forces_moments)  # propagate the MAV dynamics

    # -------update viewer-------------
    mav_view.update(mav.true_state)  # plot body of MAV
    data_view.update(mav.true_state,  # true states
                     mav.true_state,  # estimated states
                     mav.true_state,  # commanded states
                     delta,  # inputs to the aircraft
                     SIM.ts_simulation)
    if VIDEO is True:
        video.update(sim_time)

    # -------increment time-------------
    sim_time += SIM.ts_simulation

if VIDEO is True:
    video.close()



# # initialize the visualization
# mav_view = MavViewer()  # initialize the mav viewer

# # initialize elements of the architecture
# state = MsgState()

# # initialize the simulation time
# sim_time = SIM.start_time
# motions_time = 0
# time_per_motion = 3

# # main simulation loop
# while sim_time < SIM.end_time:
#     # -------vary states to check viewer-------------
#     if motions_time < time_per_motion:
#         state.north += 10*SIM.ts_simulation
#     elif motions_time < time_per_motion*2:
#         state.east += 10*SIM.ts_simulation
#     elif motions_time < time_per_motion*3:
#         state.altitude += 10*SIM.ts_simulation
#     elif motions_time < time_per_motion*4:
#         state.psi += 0.1*SIM.ts_simulation
#     elif motions_time < time_per_motion*5:
#         state.theta += 0.1*SIM.ts_simulation
#     else:
#         state.phi += 0.1*SIM.ts_simulation
#     # -------update viewer and video-------------
#     mav_view.update(state)

#     # -------increment time-------------
#     sim_time += SIM.ts_simulation
#     motions_time += SIM.ts_simulation
#     if motions_time >= time_per_motion*6:
#         motions_time = 0

#     # -------update video---------------
#     if VIDEO is True:
#         video.update(sim_time)

# if VIDEO is True:
#     video.update(sim_time)

# print("Press Ctrl-Q to exit...")