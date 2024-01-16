"""
mavsim_python
    - Chapter 13 assignment for Beard & McLain, PUP, 2012
    - Last Update:
        3/30/2022 - RWB
        1/5/2023 - David L. Christiansen
        7/13/2023 - RWB
"""
import os, sys
# insert parent directory at beginning of python search path
from pathlib import Path
sys.path.insert(0,os.fspath(Path(__file__).parents[1]))
# use QuitListener for Linux or PC <- doesn't work on Mac
#from tools.quit_listener import QuitListener
import pyqtgraph as pg
import parameters.simulation_parameters as SIM
import parameters.planner_parameters as PLAN
from models.wind_simulation import WindSimulation
from models.camera import Camera
from models.target_dynamics import TargetDynamics
from models.mav_dynamics_camera import MavDynamics
from models.gimbal import Gimbal
from control.autopilot import Autopilot
from estimation.observer import Observer
from estimation.geolocation import Geolocation
from viewers.geolocation_viewer import GeolocationViewer
from planning.path_planner import PathPlanner
from planning.path_follower import PathFollower
from planning.path_manager import PathManager
from viewers.data_viewer import DataViewer
from viewers.mav_world_camera_viewer import MAVWorldCameraViewer
from viewers.camera_viewer import CameraViewer
from message_types.msg_world_map import MsgWorldMap
from message_types.msg_waypoints import MsgWaypoints

#quitter = QuitListener()

VIDEO = False
DATA_PLOTS = False
ANIMATION = True
PLANNING_VIEWER = True
GEO_PLOTS = True
CAMERA_VIEW = True

# video initialization
if VIDEO is True:
    from viewers.video_writer import VideoWriter
    video = VideoWriter(video_name="chap13_video.avi",
                        bounding_box=(0, 0, 1000, 1000),
                        output_rate=SIM.ts_video)

# initialize the visualization
if ANIMATION or DATA_PLOTS or GEO_PLOTS:
    app = pg.QtWidgets.QApplication([]) # use the same main process for Qt applications
if ANIMATION:
    world_view = MAVWorldCameraViewer(app=app)  # initialize the viewer
if DATA_PLOTS:
    data_view = DataViewer(app=app,dt=SIM.ts_simulation, plot_period=SIM.ts_plot_refresh, 
                           data_recording_period=SIM.ts_plot_record_data, time_window_length=30)
if GEO_PLOTS:
    geo_viewer = GeolocationViewer(app=app,dt=SIM.ts_simulation, plot_period=SIM.ts_plot_refresh, 
                           data_recording_period=SIM.ts_plot_record_data, time_window_length=30)
if CAMERA_VIEW:
    camera_view = CameraViewer()

# initialize elements of the architecture
world_map = MsgWorldMap()
mav = MavDynamics(SIM.ts_simulation)
gimbal = Gimbal()
camera = Camera()
target = TargetDynamics(SIM.ts_simulation, world_map)
wind = WindSimulation(SIM.ts_simulation)
autopilot = Autopilot(SIM.ts_simulation)
observer = Observer(SIM.ts_simulation)
path_follower = PathFollower()
path_manager = PathManager()
path_planner = PathPlanner(app)

# initialize the simulation time
sim_time = SIM.start_time
plot_timer = 0

# main simulation loop
print("Press Command-Q to exit...")
while sim_time < SIM.end_time:
    # -------observer-------------
    measurements = mav.sensors()  # get sensor measurements
    camera.updateProjectedPoints(mav.true_state, target.position())
    pixels = camera.getPixels()
    #estimated_state = observer.update(measurements)  # estimate states from measurements
        # I occasionally get bad results with the observer.  true states seem to always work.
    estimated_state = mav.true_state

    # -------path planner - ----
    if path_manager.manager_requests_waypoints is True:
        waypoints = path_planner.update(world_map, estimated_state, PLAN.R_min)

    # -------path manager-------------
    path = path_manager.update(waypoints, PLAN.R_min, estimated_state)

    # -------path follower-------------
    autopilot_commands = path_follower.update(path, estimated_state)

    # -------camera control-------------
    #gimbal_cmd = gimbal.pointAtGround(estimated_state)  # point gimbal at ground
    gimbal_cmd = gimbal.pointAtPosition(estimated_state, target.position()) # point gimbal at target position

    # -------autopilot-------------
    delta, commanded_state = autopilot.update(autopilot_commands, estimated_state)
    delta.gimbal_az = gimbal_cmd.item(0)
    delta.gimbal_el = gimbal_cmd.item(1)

    # -------physical system-------------
    current_wind = wind.update()  # get the new wind vector
    mav.update(delta, current_wind)  # propagate the MAV dynamics
    target.update()  # propagate the target dynamics

    # -------update viewer-------------
    if ANIMATION:
        world_view.update(mav.true_state,
                          target.position(),
                          path, waypoints, world_map)  # plot world
    if DATA_PLOTS:
        plot_time = sim_time
        data_view.update(mav.true_state,  # true states
                         estimated_state,  # estimated states
                         commanded_state,  # commanded states
                         delta)  # inputs to aircraft

    if ANIMATION or DATA_PLOTS or PLANNING_VIEWER:
        app.processEvents()

    # -------Check to Quit the Loop-------
    # if quitter.check_quit():
    #     break

    # -------increment time-------------
    sim_time += SIM.ts_simulation





