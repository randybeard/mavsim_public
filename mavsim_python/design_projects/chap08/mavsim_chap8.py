"""
mavsim_python
    - Chapter 8 assignment for Beard & McLain, PUP, 2012
    - Last Update:
        2/21/2019 - RWB
        2/24/2020 - RWB
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
from tools.signals import Signals
from models.mav_dynamics_sensors import MavDynamics
from models.wind_simulation import WindSimulation
from control.autopilot import Autopilot
from estimation.observer import Observer
# from estimation.observer_full import Observer
from viewers.mav_viewer import MavViewer
from viewers.data_viewer import DataViewer
from viewers.sensor_viewer import SensorViewer

#quitter = QuitListener()

VIDEO = False
DATA_PLOTS = True
SENSOR_PLOTS = False
ANIMATION = True
SAVE_PLOT_IMAGE = False
COMPUTE_MODEL = False

# video initialization
if VIDEO is True:
    from viewers.video_writer import VideoWriter
    video = VideoWriter(video_name="chap8_video.avi",
                        bounding_box=(0, 0, 1000, 1000),
                        output_rate=SIM.ts_video)

#initialize the visualization
if ANIMATION or DATA_PLOTS or SENSOR_PLOTS:
    app = pg.QtWidgets.QApplication([]) # use the same main process for Qt applications
if ANIMATION:
    mav_view = MavViewer(app=app)  # initialize the mav viewer
if DATA_PLOTS:
    data_view = DataViewer(app=app,dt=SIM.ts_simulation, plot_period=SIM.ts_plot_refresh, 
                           data_recording_period=SIM.ts_plot_record_data, time_window_length=30)
if SENSOR_PLOTS:
    sensor_view = SensorViewer(app=app,dt=SIM.ts_simulation, plot_period=SIM.ts_plot_refresh, 
                           data_recording_period=SIM.ts_plot_record_data, time_window_length=30)

# initialize elements of the architecture
wind = WindSimulation(SIM.ts_simulation)
mav = MavDynamics(SIM.ts_simulation)
autopilot = Autopilot(SIM.ts_simulation)
observer = Observer(SIM.ts_simulation)

# autopilot commands
from message_types.msg_autopilot import MsgAutopilot
commands = MsgAutopilot()
Va_command = Signals(dc_offset=25.0,
                     amplitude=3.0,
                     start_time=2.0,
                     frequency = 0.01)
h_command = Signals(dc_offset=100.0,
                    amplitude=20.0,
                    start_time=0.0,
                    frequency=0.02)
chi_command = Signals(dc_offset=np.radians(0.0),
                      amplitude=np.radians(45.0),
                      start_time=10.0,
                      frequency=0.015)

# initialize the simulation time
sim_time = SIM.start_time
end_time = 100

# main simulation loop
print("Press 'Esc' to exit...")
while sim_time < end_time:

    # -------autopilot commands-------------
    commands.airspeed_command = Va_command.polynomial(sim_time)
    commands.course_command = chi_command.polynomial(sim_time)
    commands.altitude_command = h_command.polynomial(sim_time)

    # -------- autopilot -------------
    measurements = mav.sensors()  # get sensor measurements
    estimated_state = observer.update(measurements)  # estimate states from measurements
    delta, commanded_state = autopilot.update(commands, estimated_state)

    # -------- physical system -------------
    current_wind = wind.update()  # get the new wind vector
    mav.update(delta, current_wind)  # propagate the MAV dynamics

    # -------- update viewer -------------
    if ANIMATION:
        mav_view.update(mav.true_state)  # plot body of MAV
    if DATA_PLOTS:
        plot_time = sim_time
        data_view.update(mav.true_state,  # true states
                         estimated_state,  # estimated states
                         None,  # commanded states
                         delta)  # inputs to aircraft
    if SENSOR_PLOTS:
        sensor_view.update(measurements)
    if ANIMATION or DATA_PLOTS or SENSOR_PLOTS:
        app.processEvents()
    if VIDEO is True:
        video.update(sim_time)
        
    # -------Check to Quit the Loop-------
    # if quitter.check_quit():
    #     break

    # -------increment time-------------
    sim_time += SIM.ts_simulation

# Save an Image of the Plot
if SAVE_PLOT_IMAGE:
    if DATA_PLOTS:
        data_view.save_plot_image("ch8_data_plot")
    if SENSOR_PLOTS:
        sensor_view.save_plot_image("ch8_sensor_plot")

if VIDEO is True:
    video.close()




