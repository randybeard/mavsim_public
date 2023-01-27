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
import pyqtgraph as pg
import parameters.simulation_parameters as SIM
from viewers.mav_viewer import MavViewer
from viewers.data_viewer import DataViewer
from message_types.msg_delta import MsgDelta
from models.mav_dynamics_forces import MavDynamics
from tools.quit_listener import QuitListener

quitter = QuitListener()

#Running Parameters
VIDEO = False
PLOTS = True
ANIMATION = True
SAVE_PLOT_IMAGE = False

#initialize the viewers
if ANIMATION or PLOTS:
    app = pg.QtWidgets.QApplication([]) # use the same main process for Qt applications
if ANIMATION:
    mav_view = MavViewer(app=app)  # initialize the mav viewer
if PLOTS:
    # initialize view of data plots
    data_view = DataViewer(app=app,dt=SIM.ts_simulation, plot_period=SIM.ts_plot_refresh, 
                           data_recording_period=SIM.ts_plot_record_data, time_window_length=30)
if VIDEO is True:
    from viewers.video_writer import VideoWriter
    video = VideoWriter(video_name="chap3_video.avi",
                        bounding_box=(0, 0, 1000, 1000),
                        output_rate=SIM.ts_video)
    
# initialize elements of the architecture
mav = MavDynamics(SIM.ts_simulation)
delta = MsgDelta()

# initialize the simulation time
sim_time = SIM.start_time
end_time = 60

# main simulation loop
print("Press 'Esc' to exit...")
while sim_time < end_time:
    # ------- vary forces and moments to check dynamics -------------
    fx = 0  # 10
    fy = 0  # 10
    fz = 100  # 10
    Mx = 0.1  # 0.1
    My = 0  # 0.1
    Mz = 0  # 0.1
    forces_moments = np.array([[fx, fy, fz, Mx, My, Mz]]).T

    # ------- physical system -------------
    mav.update(forces_moments)  # propagate the MAV dynamics

    # ------- update viewers -------------
    if ANIMATION:
        mav_view.update(mav.true_state)  # plot body of MAV
    if PLOTS:
        data_view.update(mav.true_state,  # true states
                        None,  # estimated states
                        None,  # commanded states
                        None)  # inputs to the aircraft
    if ANIMATION or PLOTS:
        app.processEvents()
    if VIDEO is True:
        video.update(sim_time)

    # ------- increment time -------------
    sim_time += SIM.ts_simulation

    # -------Check to Quit the Loop-------
    if quitter.check_quit():
        break

# Save an Image of the Plot
if SAVE_PLOT_IMAGE and PLOTS:
    data_view.save_plot_image("ch3_plot")

# Close Video
if VIDEO is True:
    video.close()