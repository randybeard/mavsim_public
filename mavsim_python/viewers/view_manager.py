"""
mavsim: manage_viewers
    - Beard & McLain, PUP, 2012
    - Update history:
        3/11/2024 - RWB
"""
import pyqtgraph as pg
from viewers.mav_viewer import MavViewer
from viewers.mav_path_viewer import MavAndPathViewer
from viewers.data_viewer import DataViewer
from viewers.sensor_viewer import SensorViewer
import parameters.simulation_parameters as SIM
from message_types.msg_state import MsgState
from message_types.msg_delta import MsgDelta
from message_types.msg_sensors import MsgSensors
from message_types.msg_path import MsgPath

class ViewManager:
    def __init__(self, 
                 video: bool=False, 
                 data: bool=False, 
                 sensors: bool=False, 
                 animation: bool=False,
                 save_plots: bool=False,
                 path: bool=False,
                 video_name: str=[]):
        self.video_flag = video
        self.data_plot_flag = data
        self.sensor_plot_flag = sensors
        self.animation_flag = animation
        self.path_flag = path
        self.save_plots_flag = save_plots
        # initialize video 
        if self.video_flag is True:
            from viewers.video_writer import VideoWriter
            self.video = VideoWriter(
                video_name=video_name,
                bounding_box=(0, 0, 1000, 1000),
                output_rate=SIM.ts_video)
        # initialize the other visualization
        if self.animation_flag or self.data_plot_flag or self.sensor_plot_flag: 
            self.app = pg.QtWidgets.QApplication([]) 
            if self.animation_flag:
                if self.path_flag:
                    self.mav_view = MavAndPathViewer(app=self.app)
                else:
                    self.mav_view = MavViewer(app=self.app)  
            if self.data_plot_flag: 
                self.data_view = DataViewer(
                    app=self.app,
                    dt=SIM.ts_simulation,
                    plot_period=SIM.ts_plot_refresh, 
                    data_recording_period=SIM.ts_plot_record_data, 
                    time_window_length=30)
            if self.sensor_plot_flag: 
                self.sensor_view = SensorViewer(
                    app=self.app,
                    dt=SIM.ts_simulation, 
                    plot_period=SIM.ts_plot_refresh, 
                    data_recording_period=SIM.ts_plot_record_data, 
                    time_window_length=30)

    def update(self,
               sim_time: float,
               true_state: MsgState, 
               estimated_state: MsgState, 
               commanded_state: MsgState, 
               delta: MsgDelta,
               measurements: MsgSensors=None,
               path: MsgPath=None):
        if self.animation_flag: 
            if self.path_flag is True:
                self.mav_view.update(true_state, path)
            else:
                self.mav_view.update(true_state) 
        if self.data_plot_flag:
            self.data_view.update(
                true_state,  # true states
                estimated_state,  # estimated states
                commanded_state,  # commanded states
                delta)  # inputs to aircraft
        if self.sensor_plot_flag: 
            self.sensor_view.update(measurements)
        if self.animation_flag or self.data_plot_flag or self.sensor_plot_flag: 
            self.app.processEvents()
        if self.video_flag is True: 
            self.video.update(sim_time)
    
    def close(self, dataplot_name: str=[], sensorplot_name: str=[]):
        # Save an Image of the Plot
        if self.save_plots_flag:
            if self.data_plots_flag: 
                self.data_view.save_plot_image(dataplot_name)
            if self.sensor_plots_flag: 
                self.sensor_view.save_plot_image(sensorplot_name)
        if self.video_flag: 
            self.video.close()

