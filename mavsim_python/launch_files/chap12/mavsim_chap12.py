"""
mavsim_python
    - Chapter 12 assignment for Beard & McLain, PUP, 2012
    - Last Update:
        4/3/2019 - BGM
        2/27/2020 - RWB
        1/5/2023 - David L. Christiansen
        7/13/2023 - RWB
        4/1/2024 - RWB
"""
import os, sys
# insert parent directory at beginning of python search path
from pathlib import Path
sys.path.insert(0,os.fspath(Path(__file__).parents[2]))
# use QuitListener for Linux or PC <- doesn't work on Mac
#from python_tools.quit_listener import QuitListener
import parameters.simulation_parameters as SIM
import parameters.planner_parameters as PLAN
from models.mav_dynamics_sensors import MavDynamics
from models.wind_simulation import WindSimulation
from controllers.autopilot import Autopilot
from estimators.observer import Observer
#from estimators.observer_full import Observer
from planners.path_follower import PathFollower
from planners.path_manager import PathManager
from planners.path_planner import PathPlanner
from viewers.view_manager import ViewManager
import time
from message_types.msg_world_map import MsgWorldMap
#quitter = QuitListener()

# initialize elements of the architecture
mav = MavDynamics(SIM.ts_simulation)
wind = WindSimulation(SIM.ts_simulation)
autopilot = Autopilot(SIM.ts_simulation)
observer = Observer(SIM.ts_simulation)
path_follower = PathFollower()
path_manager = PathManager()
#planner_type = 'simple_straight'  # return simple waypoint path
#planner_type = 'simple_dubins'  # return simple dubins waypoint path
#planner_type = 'rrt_straight'  # plan path through city using straight-line RRT
planner_type = 'rrt_dubins'  # plan path through city using dubins RRT
path_planner = PathPlanner(type=planner_type)
viewers = ViewManager(map=True,
                      planning=True, 
                      video=False, video_name='chap12.mp4')
world_map = MsgWorldMap()

# initialize the simulation time
sim_time = SIM.start_time
end_time = 200

# main simulation loop
print("Press 'Esc' to exit...")
while sim_time < SIM.end_time:
    # -------observer-------------
    measurements = mav.sensors()  # get sensor measurements
    estimated_state = observer.update(measurements)  # estimate states from measurements
    #estimated_state = mav.true_state

    # -------path planner - ----
    if path_manager.manager_requests_waypoints is True:
        waypoints = path_planner.update(world_map, estimated_state, PLAN.R_min)
        # this update shows the intermediary tree
        viewers.update_planning_tree(
               waypoints=waypoints,
               map=world_map,
               waypoints_not_smooth=path_planner.waypoints_not_smooth,
               tree=path_planner.tree,
               radius=PLAN.R_min,
               )

    # -------path manager-------------
    path = path_manager.update(waypoints, estimated_state, PLAN.R_min)

    # -------path follower-------------
    autopilot_commands = path_follower.update(path, estimated_state)

    # -------autopilot-------------
    delta, commanded_state = autopilot.update(autopilot_commands, estimated_state)

    # -------physical system-------------
    current_wind = wind.update()  # get the new wind vector
    mav.update(delta, current_wind)  # propagate the MAV dynamics

    # -------update viewer-------------
    viewers.update(
        sim_time,
        true_state=mav.true_state,  # true states
        estimated_state=estimated_state,  # estimated states
        commanded_state=commanded_state,  # commanded states
        delta=delta,  # inputs to aircraft
        path=path, # path
        waypoints=waypoints, # waypoints
        map=world_map,  # map of world
    )

    # -------Check to Quit the Loop-------
    # if quitter.check_quit():
    #     break

    # -------increment time-------------
    sim_time += SIM.ts_simulation

# close viewers
viewers.close(dataplot_name="ch12_data_plot")



