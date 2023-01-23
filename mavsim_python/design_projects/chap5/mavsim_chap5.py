"""
mavsim_python
    - Chapter 5 assignment for Beard & McLain, PUP, 2012
    - Last Update:
        2/2/2019 - RWB
"""
import sys
sys.path.append('../..')
import numpy as np
import parameters.simulation_parameters as SIM
from viewers.mav_viewer import MavViewer
from viewers.data_viewer import DataViewer
from models.mav_dynamics_control import MavDynamics
from models.wind_simulation import WindSimulation
from models.trim import compute_trim
from models.compute_models import compute_model
from tools.signals import Signals
import pyqtgraph as pg

VIDEO = False
PLOTS = True
ANIMATION = True
SAVE_PLOT_IMAGE = False

# video initialization
if VIDEO is True:
    from viewers.video_writer import VideoWriter
    video = VideoWriter(video_name="chap5_video.avi",
                        bounding_box=(0, 0, 1000, 1000),
                        output_rate=SIM.ts_video)

#initialize the visualization
app = pg.QtWidgets.QApplication([]) # use the same main process for Qt applications
if ANIMATION:
    mav_view = MavViewer(app=app)  # initialize the mav viewer
if PLOTS:
    data_view = DataViewer(app=app, dt=SIM.ts_simulation, plot_period=SIM.ts_plotting)

# initialize elements of the architecture
wind = WindSimulation(SIM.ts_simulation)
mav = MavDynamics(SIM.ts_simulation)

# use compute_trim function to compute trim state and trim input
Va = 25.
gamma = 0.*np.pi/180.
trim_state, trim_input = compute_trim(mav, Va, gamma)
mav._state = trim_state  # set the initial state of the mav to the trim state
delta = trim_input  # set input to constant constant trim input

# # compute the state space model linearized about trim
compute_model(mav, trim_state, trim_input)

# this signal will be used to excite modes
input_signal = Signals(amplitude=0.3,
                       duration=0.3,
                       start_time=5.0)
delta_e_trim = delta.elevator
delta_a_trim = delta.aileron
delta_r_trim = delta.rudder

# initialize the simulation time
sim_time = SIM.start_time

# main simulation loop
print("Press CTRL-C to exit...")
while sim_time < SIM.end_time:

    # -------physical system-------------
    #current_wind = wind.update()  # get the new wind vector
    current_wind = np.zeros((6, 1))
    # this input excites the phugoid mode by adding an elevator impulse at t = 5.0 s
    # delta.elevator = delta_e_trim + input_signal.impulse(sim_time)
    # this input excites the roll and spiral divergence modes by adding an aileron doublet at t = 5.0 s
    # delta.aileron = delta_a_trim + input_signal.doublet(sim_time)
    # this input excites the dutch roll mode by adding a rudder doublet at t = 5.0 s
    # delta.rudder = delta_r_trim + input_signal.doublet(sim_time)

    mav.update(delta, current_wind)  # propagate the MAV dynamics

    # -------update viewer-------------
    if ANIMATION:
        mav_view.update(mav.true_state)  # plot body of MAV
    if PLOTS:
        plot_time = sim_time
        data_view.update(mav.true_state,  # true states
                            mav.true_state,  # estimated states
                            mav.true_state,  # commanded states
                            delta)  # inputs to aircraft

    if ANIMATION or PLOTS:
        app.processEvents()

    # -------increment time-------------
    sim_time += SIM.ts_simulation

if SAVE_PLOT_IMAGE:
    data_view.save_plot_image("ch5_plot")

if VIDEO is True:
    video.close()



