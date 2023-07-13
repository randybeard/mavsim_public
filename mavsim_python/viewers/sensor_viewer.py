"""
sensor_viewer

part of mavsimPy
    - Beard & McLain, PUP, 2012
    - Update history:
        12/17/2018 - RWB
        1/14/2019 - RWB
        2/27/2020 - RWB
        1/19/2023 - DLC
"""
from plotter.plotter import Plotter
import numpy as np

class SensorViewer:
    def __init__(self, app, dt = 0.01,
                 time_window_length = 50, # number of data points plotted at a time
                 plot_period=2, # time interval between a plot update
                 data_recording_period=0.05): # time interval between a data record
        self._dt = dt
        self._data_window_length= time_window_length/data_recording_period
        self._update_counter = 0
        self._plots_per_row = 4
        self._plotter = Plotter(app=app,plots_per_row=self._plots_per_row)  # plot last time_window seconds of data
        self._plot_period = plot_period
        self._data_recording_period = data_recording_period
        self._plot_delay = 0
        self._data_recording_delay = 0
        self._time = 0

        #define colors
        gps_color = (0,255,0)
        gyro_color = (255,0,0)
        pressure_color = (0,0,255)
        accelerometer_color = (255,154,111)

        # define first row
        self._plotter.create_plot_widget(plot_id='gyro_x', xlabel='Time (s)', ylabel='gyro_x(m/s)',
                                        window_length=self._data_window_length)
        self._plotter.create_plot_widget(plot_id='gyro_y', xlabel='Time (s)', ylabel='gyro_y(m/s)',
                                       window_length=self._data_window_length)
        self._plotter.create_plot_widget(plot_id='gyro_z', xlabel='Time (s)', ylabel='gyro_z(m/s)',
                                       window_length=self._data_window_length)
        self._plotter.create_plot_widget(plot_id='absolute_pressure', xlabel='Time (s)', ylabel='pressure(Pa)',
                                       window_length=self._data_window_length)
        self._plotter.create_data_set(plot_id='gyro_x', data_label='gyro_x', data_color=gyro_color)
        self._plotter.create_data_set(plot_id='gyro_y', data_label='gyro_y', data_color=gyro_color) 
        self._plotter.create_data_set(plot_id='gyro_z', data_label='gyro_z', data_color=gyro_color)
        self._plotter.create_data_set(plot_id='absolute_pressure', data_label='abs_pressure', data_color=pressure_color)

        # define second row
        self._plotter.create_plot_widget(plot_id='accel_x', xlabel='Time (s)', ylabel='accel_x(m/s^2)',
                                       window_length=self._data_window_length)
        self._plotter.create_plot_widget(plot_id='accel_y', xlabel='Time (s)', ylabel='accel_y(m/s^2)',
                                       window_length=self._data_window_length)
        self._plotter.create_plot_widget(plot_id='accel_z', xlabel='Time (s)', ylabel='accel_z(m/s^2)',
                                       window_length=self._data_window_length)
        self._plotter.create_plot_widget(plot_id='diff_pressure', xlabel='Time (s)', ylabel='pressure(Pa)',
                                       window_length=self._data_window_length)
        self._plotter.create_data_set(plot_id='accel_x', data_label='accel_x', data_color=accelerometer_color)
        self._plotter.create_data_set(plot_id='accel_y', data_label='accel_y', data_color=accelerometer_color)
        self._plotter.create_data_set(plot_id='accel_z', data_label='accel_z', data_color=accelerometer_color)
        self._plotter.create_data_set(plot_id='diff_pressure', data_label='diff_pressure', data_color=pressure_color)

        # define third row
        self._plotter.create_plot_widget(plot_id='gps_n', xlabel='Time (s)', ylabel='distance(m)',
                                       window_length=self._data_window_length)
        self._plotter.create_plot_widget(plot_id='gps_e', xlabel='Time (s)', ylabel='distance(m)',
                                       window_length=self._data_window_length)
        self._plotter.create_plot_widget(plot_id='gps_h', xlabel='Time (s)', ylabel='distance(m)',
                                       window_length=self._data_window_length)
        self._plotter.create_plot_widget(plot_id='gps_Vg', xlabel='Time (s)', ylabel='gps_Vg(m/s)',
                                       window_length=self._data_window_length)
        self._plotter.create_data_set(plot_id='gps_n', data_label='gps_n', data_color=gps_color)
        self._plotter.create_data_set(plot_id='gps_e', data_label='gps_e', data_color=gps_color)
        self._plotter.create_data_set(plot_id='gps_h', data_label='gps_h', data_color=gps_color)
        self._plotter.create_data_set(plot_id='gps_Vg', data_label='gps_Vg', data_color=gps_color)

        # define fourth row
        self._plotter.create_plot_widget(plot_id='gps_course', xlabel='Time (s)', ylabel='gps_course (deg)',
                                       window_length=self._data_window_length)
        self._plotter.create_data_set(plot_id='gps_course', data_label='gps_course', data_color=gps_color)
        self._plotter.show_window()

    def update(self, sensors):
        if self._data_recording_delay >= self._data_recording_period:
            self.__update_data(sensors, self._time)
            self._data_recording_delay = 0
        if self._plot_delay >= self._plot_period:
            self.__update_plot()
            self._plot_delay = 0
        self._plot_delay += self._dt
        self._data_recording_delay += self._dt
        self._time += self._dt
        
    def __update_data(self, sensors, t):
        #add the commanded state data
        self._plotter.add_data_point(plot_id='gyro_x', data_label='gyro_x', xvalue=t, yvalue=sensors.gyro_x)
        self._plotter.add_data_point(plot_id='gyro_y', data_label='gyro_y', xvalue=t, yvalue=sensors.gyro_y)
        self._plotter.add_data_point(plot_id='gyro_z', data_label='gyro_z', xvalue=t, yvalue=sensors.gyro_z)
        self._plotter.add_data_point(plot_id='absolute_pressure', data_label='abs_pressure',xvalue=t, yvalue=sensors.abs_pressure)
        self._plotter.add_data_point(plot_id='accel_x', data_label='accel_x', xvalue=t, yvalue=sensors.accel_x)
        self._plotter.add_data_point(plot_id='accel_y', data_label='accel_y', xvalue=t, yvalue=sensors.accel_y)
        self._plotter.add_data_point(plot_id='accel_z', data_label='accel_z', xvalue=t, yvalue=sensors.accel_z)
        self._plotter.add_data_point(plot_id='diff_pressure',   data_label='diff_pressure', xvalue=t, yvalue=sensors.diff_pressure)
        self._plotter.add_data_point(plot_id='gps_n', data_label='gps_n', xvalue=t, yvalue=sensors.gps_n)
        self._plotter.add_data_point(plot_id='gps_e', data_label='gps_e', xvalue=t, yvalue=sensors.gps_e)
        self._plotter.add_data_point(plot_id='gps_h', data_label='gps_h', xvalue=t, yvalue=sensors.gps_h)
        self._plotter.add_data_point(plot_id='gps_Vg', data_label='gps_Vg', xvalue=t, yvalue=sensors.gps_Vg)
        self._plotter.add_data_point(plot_id='gps_course', data_label='gps_course', xvalue=t, yvalue=self.__rad_to_deg(sensors.gps_course))

    def process_app(self):
        self._plotter.process_app(0)

    def __update_plot(self):
        self._plotter.update_plots()

    def close_sensor_viewer(self):
        self._plotter.close_window()

    def save_plot_image(self, plot_name):
        self._plotter.save_image(plot_name)

    def __rad_to_deg(self, radians):
        return radians*180/np.pi
