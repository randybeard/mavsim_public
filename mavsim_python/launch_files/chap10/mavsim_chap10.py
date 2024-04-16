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
sys.path.insert(0,os.fspath(Path(__file__).parents[2]))
# use QuitListener for Linux or PC <- doesn't work on Mac
#from tools.quit_listener import QuitListener
import numpy as np
import parameters.simulation_parameters as SIM
from models.mav_dynamics_sensors import MavDynamics
from models.wind_simulation import WindSimulation
from controllers.autopilot import Autopilot
from estimators.observer_full import Observer
#from estimators.observer import Observer
from planners.path_follower import PathFollower
from viewers.view_manager import ViewManager
from message_types.msg_path import MsgPath
import time

# initialize elements of the architecture
wind = WindSimulation(SIM.ts_simulation)
mav = MavDynamics(SIM.ts_simulation)
autopilot = Autopilot(SIM.ts_simulation)
observer = Observer(SIM.ts_simulation)
path_follower = PathFollower()
viewers = ViewManager(path=True, 
                      data=False,
                      video=False, video_name='chap10.mp4')
#quitter = QuitListener()

# path definition
# path = MsgPath(
#     type='line', 
#     airspeed = 25,
#     line_origin = np.array([[0.0, 0.0, -100.0]]).T,
#     line_direction = np.array([[0.5, 1.0, 0.0]]).T,
#     )
path = MsgPath(
    type='orbit', 
    airspeed = 25,
    orbit_center = np.array([[0.0, 0.0, -100.0]]).T,
    orbit_radius = 200.0,
    orbit_direction = 'CCW', # 'CCW',
    )
# path = MsgPath(
#     type='helix', 
#     airspeed = 25,
#     orbit_center = np.array([[0.0, 0.0, -100.0]]).T,
#     orbit_radius = 200.0,
#     orbit_direction = 'CW', # 'CCW',
#     helix_climb_angle=np.radians(1),
#     helix_start_angle=np.radians(0),
#     )

# initialize the simulation time
sim_time = SIM.start_time
end_time = 300

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

    # -------- update viewer -------------
    viewers.update(
        sim_time,
        true_state=mav.true_state,  # true states
        estimated_state=estimated_state,  # estimated states        
        commanded_state=commanded_state,  # commanded states
        delta=delta, # inputs to MAV
        path=path, # path
    )
        
    # -------Check to Quit the Loop-------
    # if quitter.check_quit():
    #     break

    # -------increment time-------------
    sim_time += SIM.ts_simulation
    #time.sleep(0.001)  # make the sim run slower

# close viewers
viewers.close(dataplot_name="ch10_data_plot")





