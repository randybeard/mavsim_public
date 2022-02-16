from state_plotter.Plotter import Plotter
from state_plotter.plotter_args import *


class SensorViewer:
    def __init__(self):
        time_window_length=100
        self.plotter = Plotter(plotting_frequency=100,  # refresh plot every 100 time steps
                               time_window=time_window_length,  # plot last time_window seconds of data
                               window_title="Sensors")
        # set up the plot window
        # define first row
        gyro_x_plots = PlotboxArgs(plots=['gyro_x'],
                               labels={'left': 'gyro_x(m/s)', 'bottom': 'Time (s)'},
                               time_window=time_window_length)
        gyro_y_plots = PlotboxArgs(plots=['gyro_y'],
                                   labels={'left': 'gyro_y(m/s)', 'bottom': 'Time (s)'},
                                   time_window=time_window_length)
        gyro_z_plots = PlotboxArgs(plots=['gyro_z'],
                                   labels={'left': 'gyro_z(m/s)', 'bottom': 'Time (s)'},
                                   time_window=time_window_length)
        abs_pressure_plots = PlotboxArgs(plots=['absolute_pressure'],
                                            labels={'left': 'pressure(Pa)', 'bottom': 'Time (s)'},
                                            time_window=time_window_length)

        first_row = [gyro_x_plots, gyro_y_plots, gyro_z_plots, abs_pressure_plots]

        # define second row
        accel_x_plots = PlotboxArgs(plots=['accel_x'],
                                    labels={'left': 'accel_x(m/s^2)', 'bottom': 'Time (s)'},
                                    time_window=time_window_length)
        accel_y_plots = PlotboxArgs(plots=['accel_y'],
                                    labels={'left': 'accel_y(m/s^2)', 'bottom': 'Time (s)'},
                                    time_window=time_window_length)
        accel_z_plots = PlotboxArgs(plots=['accel_z'],
                                    labels={'left': 'accel_z(m/s^2)', 'bottom': 'Time (s)'},
                                    time_window=time_window_length)
        diff_pressure_plots = PlotboxArgs(plots=['diff_pressure'],
                                          labels={'left': 'pressure(Pa)', 'bottom': 'Time (s)'},
                                          time_window=time_window_length)
        second_row = [accel_x_plots, accel_y_plots, accel_z_plots, diff_pressure_plots]

        # define third row
        gps_n_plots = PlotboxArgs(plots=['gps_n'],
                                  labels={'left': 'distance(m)', 'bottom': 'Time (s)'},
                                  time_window=time_window_length)
        gps_e_plots = PlotboxArgs(plots=['gps_e'],
                                  labels={'left': 'distance(m)', 'bottom': 'Time (s)'},
                                  time_window=time_window_length)
        gps_h_plots = PlotboxArgs(plots=['gps_h'],
                                  labels={'left': 'distance(m)', 'bottom': 'Time (s)'},
                                  time_window=time_window_length)
        third_row = [gps_n_plots, gps_e_plots, gps_h_plots]

        # define fourth row
        gps_Vg_plots = PlotboxArgs(plots=['gps_Vg'],
                                   labels={'left': 'gps_Vg(m/s)', 'bottom': 'Time (s)'},
                                   time_window=time_window_length)
        gps_course_plots = PlotboxArgs(plots=['gps_course'],
                                       labels={'left': 'gps_course (deg)', 'bottom': 'Time (s)'},
                                       rad2deg=True,
                                       time_window=time_window_length)

        fourth_row = [gps_Vg_plots, gps_course_plots]
        plots = [first_row,
                 second_row,
                 third_row,
                 fourth_row
                 ]
        # Add plots to the window
        self.plotter.add_plotboxes(plots)
        # Define and label vectors for more convenient/natural data input
        self.plotter.define_input_vector('sensors', ['gyro_x', 'gyro_y', 'gyro_z',
                                                     'absolute_pressure',
                                                     'accel_x', 'accel_y', 'accel_z',
                                                     'diff_pressure',
                                                     'gps_n', 'gps_e', 'gps_h',
                                                     'gps_Vg', 'gps_course'])
        # plot timer
        self.time = 0.

    def update(self, sensors, ts):
        ## Add the sensors data in vectors
        # the order has to match the order in lines 72-76
        sensor_list = [sensors.gyro_x, sensors.gyro_y, sensors.gyro_z,
                       sensors.abs_pressure, sensors.accel_x, sensors.accel_y,
                       sensors.accel_z, sensors.diff_pressure, sensors.gps_n,
                       sensors.gps_e, sensors.gps_h, sensors.gps_Vg,
                       sensors.gps_course]
        self.plotter.add_vector_measurement('sensors', sensor_list, self.time)

        # Update and display the plot
        self.plotter.update_plots()

        # increment time
        self.time += ts