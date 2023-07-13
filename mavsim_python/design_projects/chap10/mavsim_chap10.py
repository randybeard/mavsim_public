"""
mavsim_python
    - Chapter 10 assignment for Beard & McLain, PUP, 2012
    - Last Update:
        3/11/2019 - RWB
        2/27/2020 - RWB
        1/5/2023 - David L. Christiansen
        7/13/2023 - RWB
"""
import os, sys
# insert parent directory at beginning of python search path
from pathlib import Path
sys.path.insert(0,os.fspath(Path(__file__).parents[1]))
# use QuitListener for Linux or PC <- doesn't work on Mac
#from tools.quit_listener import QuitListener
import numpy as np
import pyqtgraph as pg
import parameters.simulation_parameters as SIM
from models.mav_dynamics_sensors import MavDynamics
from models.wind_simulation import WindSimulation
from control.autopilot import Autopilot
from estimation.observer import Observer
from planning.path_follower import PathFollower
from viewers.data_viewer import DataViewer
from viewers.mav_path_viewer import MAVAndPathViewer

#quitter = QuitListener()

VIDEO = False
DATA_PLOTS = True
ANIMATION = True
SAVE_PLOT_IMAGE = False

# video initialization
if VIDEO is True:
    from viewers.video_writer import VideoWriter
    video = VideoWriter(video_name="chap10_video.avi",
                        bounding_box=(0, 0, 1000, 1000),
                        output_rate=SIM.ts_video)

# initialize the visualization
if ANIMATION or DATA_PLOTS:
    app = pg.QtWidgets.QApplication([]) # use the same main process for Qt applications
if ANIMATION:
    path_view = MAVAndPathViewer(app=app)  # initialize the viewer
if DATA_PLOTS:
    data_view = DataViewer(app=app,dt=SIM.ts_simulation, plot_period=SIM.ts_plot_refresh, 
                           data_recording_period=SIM.ts_plot_record_data, time_window_length=30)


# initialize elements of the architecture
wind = WindSimulation(SIM.ts_simulation)
mav = MavDynamics(SIM.ts_simulation)
autopilot = Autopilot(SIM.ts_simulation)
observer = Observer(SIM.ts_simulation)
path_follower = PathFollower()

# path definition
from message_types.msg_path import MsgPath
path = MsgPath()
path.type = 'line'
#path.type = 'orbit'
if path.type == 'line':
    path.line_origin = np.array([[0.0, 0.0, -100.0]]).T
    path.line_direction = np.array([[0.5, 1.0, 0.0]]).T
    path.line_direction = path.line_direction / np.linalg.norm(path.line_direction)
elif path.type == 'orbit':
    path.orbit_center = np.array([[0.0, 0.0, -100.0]]).T  # center of the orbit
    path.orbit_radius = 300.0  # radius of the orbit
    path.orbit_direction = 'CW'  # orbit direction: 'CW'==clockwise, 'CCW'==counter clockwise

# initialize the simulation time
sim_time = SIM.start_time
end_time = 60

# main simulation loop
print("Press 'Esc' to exit...")
while sim_time < end_time:
    # -------observer-------------
    measurements = mav.sensors()  # get sensor measurements
    estimated_state = observer.update(measurements)  # estimate states from measurements

    # -------path follower-------------
    autopilot_commands = path_follower.update(path, estimated_state)
    #autopilot_commands = path_follower.update(path, mav.true_state)  # for debugging

    # -------autopilot-------------
    delta, commanded_state = autopilot.update(autopilot_commands, estimated_state)

    # -------physical system-------------
    current_wind = wind.update()  # get the new wind vector
    mav.update(delta, current_wind)  # propagate the MAV dynamics

        # ------- animation -------
    if ANIMATION:
        path_view.update(mav.true_state, path)  # plot path and MAV
    if DATA_PLOTS:
        plot_time = sim_time
        data_view.update(mav.true_state,  # true states
                         estimated_state,  # estimated states
                         commanded_state,  # commanded states
                         delta)  # inputs to aircraft
    if ANIMATION or DATA_PLOTS:
        app.processEvents()
    if VIDEO is True:
        video.update(sim_time)
        
    # -------Check to Quit the Loop-------
    # if quitter.check_quit():
    #     break

    # -------increment time-------------
    sim_time += SIM.ts_simulation

# Save an Image of the Plot
if SAVE_PLOT_IMAGE and DATA_PLOTS:
        data_view.save_plot_image("ch10_data_plot")

if VIDEO is True:
    video.close()




