#!/usr/bin/env python
from threading import Lock
import numpy as np
from collections import defaultdict
import pyqtgraph as pg
from pyqtgraph import ViewBox
import argparse
from state_plotter.plotter_args import PlotArgs, PlotboxArgs
from state_plotter.state_plotbox import StatePlotbox
from state_plotter.state_plot import StatePlot
from state_plotter.state_data import StateData
from pdb import set_trace

# Enable antialiasing for prettier plots
pg.setConfigOptions(antialias=True)

class Plotter:
    """
    Class for plotting methods.
    """
    def __init__(self, plotting_frequency=1, time_window=15, window_title="States"):
        ''' Initialize the Plotter

            plotting_freq: number of times the update function must be called
                           until the plotter actually outputs the graph.
                           (Can help reduce the slow-down caused by plotting)

            time_window:   how many seconds of data to appear in the plot
        '''
        self.time_window = time_window
        self.time = 0
        self.prev_time = 0

        # Able to update plots intermittently for speed
        self.plotting_frequency = plotting_frequency
        self.freq_counter = 0

        # Plot default parameters
        # TODO: Migrate
        self.x_grid_on = False
        self.y_grid_on = True

        # Plot theme params -- default is dark theme
        self.background_color = 'k'
        self.axis_pen = pg.mkPen(color='w', width=1)

        # initialize Qt gui application and window
        self.default_window_size = (1000, 800)
        self.app = pg.QtGui.QApplication([])
        self.window = pg.GraphicsWindow(title=window_title)
        self.window.resize(*self.default_window_size)
        self.window.setBackground(self.background_color)
        self.old_windows = []
        self.row_plot_count = 0

        # Define plot argument parser
        self.arg_parser = self._define_plot_arg_parser()
        self.hidden_curve_prefix = '_'


        self.new_data = False
        self.plot_cnt = 0
        self.plotboxes = {}
        self.states = defaultdict(list)
        self.input_vectors = {}

        # Multi dimension support
        self.plot_dimension = {}
        self.multi_dim_auto_adjust = True
        self.multi_dim_state_delimiter = '&'
        # Circle marker for 2d plots (marks most recent data)
        # self.xy_marker_on = True
        # self.xy_marker_radius = 3
        # self.xy_marker_circle = self._get_circle((0,0), self.xy_marker_radius)
        # self.xy_marker_postfix = "_marker_"

        # asynchronous variable acess protection
        self.states_lock = Lock() # Lock to prevent asynchronous changes to states


    def define_input_vector(self, vector_name, input_vector):
        ''' Defines an input vector so measurements can be added in groups

            vector_name (string): name of the vector
            input_vector (list of strings): order of states in the vector

            Note: this does not add states or plots, so plots for the values in
            *input_vector* will need to also be added via the *add_plot* function
        '''
        self.input_vectors[vector_name] = input_vector

    def add_window(self, window_title):
        # Create a new window
        self.window = pg.GraphicsWindow(title=window_title)
        self.window.resize(*self.default_window_size)
        self.window.setBackground(self.background_color)

        # Reset plot count
        self.plot_cnt = 0

    def use_light_theme(self):
        self.background_color = 'w'
        self.window.setBackground(self.background_color)
        self.axis_pen = pg.mkPen(color='k', width=1)
        self.plot_min_hue = 360
        self.plot_max_hue = 72
        self.plot_min_value = 0
        self.plot_max_value = 180

    def set_plots_per_row(self, n):
        self.plots_per_row = n

    def set_grids(self, x_grid_on, y_grid_on):
        self.x_grid_on = x_grid_on
        self.y_grid_on = y_grid_on

    def add_plotbox(self, plotbox_args):
        ''' Adds a state and the necessary plot, curves, and data lists

            curve_names: name(s) of the state(s) to be plotted in the same plot window
                         (e.g. ['x', 'x_truth'] or ['x', 'x_command'])
        '''
        # Backwards compatibility with string definitions
        if isinstance(plotbox_args, str):
            # Parse the string for curve names and arguments
            plotbox_args = self._parse_plot_str(plotbox_args)

        self._add_plot_box(plotbox_args)

    def add_plotboxes(self, plotbox_arg_list):
        ''' Add multiple plotboxes, configured according to the structure of *plotbox_arg_list*

        Arguments:
            plotbox_arg_list (list of PlotboxArgs objects or strings): contains the arguments
                for each plotbox to be added. If the list is two-dimensional, the plotboxes
                will be added according to the list structure:

                Example:
                    [['x', 'y'],           would produce a plot with x and y on
                     ['u', 'v', 'w'],  --> the first row, u, v, and w on the 2nd,
                     ['phi']]              and phi on the 3rd.

        '''
        if isinstance(plotbox_arg_list[0], list):
            # Base row size on list length
            for row in plotbox_arg_list:
                self.set_plots_per_row(len(row))
                for plot in row:
                    self.add_plotbox(plot)
        else:
            # Use same row size for the whole window
            for plot in plotbox_arg_list:
                self.add_plotbox(plot)

    def add_vector_measurement(self, vector_name, vector_values, time, sigma_values=None, rad2deg=False):
        '''Adds a group of measurements in vector form

            vector_name (string): name given the vector through the *define_input_vector*
                                  function
            vector_values (list of numbers): values for each of the states in the
                          order defined in the *define_input_vector* function
            time: time stamp for the values in the vector
            rad2deg: Flag to convert the state value from radians to degrees

        '''
        if len(vector_values) != len(self.input_vectors[vector_name]):
            raise ValueError("State vector length mismatch. \
                          State vector '{0}' has length {1}".format(vector_name, len(vector_values)))
        if sigma_values is None:
            sigma_values = [0.]*len(vector_values)
        for i, state in enumerate(self.input_vectors[vector_name]):
            self.add_measurement(state, vector_values[i], time, sigma_values[i])


    def add_measurement(self, state_name, state_val, time, sigma=0.0):
        '''Adds a measurement for the given state

            state_name (string): name of the state
            state_val (float): value to be added for the state
            time (float): time (in seconds) of the measurement
        '''
        self.states_lock.acquire()
        for state_obj in self.states[state_name]:
            state_obj.add_data(state_val, time, sigma)
        self.states_lock.release()
        self.new_data = True
        self.time = max(self.time, time) # Keep track of the latest data point

    def set_data(self, state_name, state_vals, times, sigmas=None):
        if isinstance(times, (int,float)):
            times = [times]*len(state_vals)
        self.states_lock.acquire()
        for state_obj in self.states[state_name]:
            state_obj.set_data(state_vals, times, sigmas)
        self.states_lock.release()
        self.new_data = True
        self.time = max(self.time, times[-1])


    # Update the plots with the current data
    def update_plots(self):
        '''Update the plots (according to plotting frequency defined in init) '''
        if self.time > self.prev_time:
            # Only process data if time has changed
            self.freq_counter += 1
            if self.new_data and (self.freq_counter % self.plotting_frequency == 0):
                for pb in self.plotboxes.values():
                    self.states_lock.acquire()
                    pb.update(self.time)
                    self.states_lock.release()
                self.new_data = False
                self.prev_time = self.time
        # update the plots
        self.app.processEvents()


    #
    # Private Methods
    #

    def _add_plot_box(self, plotbox_args):
        ''' Adds a plot box to the plotting window '''
        plotbox = StatePlotbox(self.window, plotbox_args)
        if plotbox_args.title in self.plotboxes:
            raise ValueError('Plotbox with title \"{}\" already exists in the window.'\
                             .format(plotbox_args.title)\
                              + ' Cannot add duplicate.')
        self.plotboxes[plotbox_args.title] = plotbox
        self._add_states(plotbox)
        self.row_plot_count += 1
        if self.row_plot_count % self.plots_per_row == 0:
            self.window.nextRow()
            self.row_plot_count = 0

    def _add_states(self, plotbox):
        states = plotbox.get_states()
        for k,v in states.items():
            self.states[k].append(v)

    def _define_plot_arg_parser(self):
        parser = argparse.ArgumentParser()
        parser.add_argument("curves", nargs="+")
        parser.add_argument('-l', '--legend', action='store_true')
        parser.add_argument('-n', '--name', nargs="+")

        dim_group = parser.add_mutually_exclusive_group()
        dim_group.add_argument('-2d', '--2d', action='store_const', dest="dimension", const=2, default=1)
        dim_group.add_argument('-3d', '--3d', action='store_const', dest="dimension", const=3, default=1)
        return parser

    def _parse_plot_str(self, plot_str):
        args = self.arg_parser.parse_args(plot_str.split())

        # Find hidden curves
        args.hidden_curves = []
        for c in args.curves:
            if c.startswith(self.hidden_curve_prefix):
                args.hidden_curves.append(c[1:])
        # Remove from regular curves
        for c in args.hidden_curves:
            args.curves.remove("_" + c)

        dim = args.dimension
        # Check for dimension issues
        if len(args.curves) % dim != 0:
            e = "Plot string error: dimension ({0}) does not match number of curves ({1}).".format(dim, args.curves)
            raise Exception(e)

        if args.name is None and len(args.curves) > 0:
            if dim == 1:
                args.name = args.curves[0]
            else:
                args.name = self.multi_dim_state_delimiter.join(args.curves[0:dim])
        else:
            if dim > 1:
                if self.default_label_pos == 'left':
                    args.name = self.multi_dim_state_delimiter.join(['', args.name])
                else:
                    args.name = self.multi_dim_state_delimiter.join([args.name, ''])

        plots = []
        for c in np.reshape(args.curves, (-1,args.dimension)):
            plot_name = self.multi_dim_state_delimiter.join(c) # If dim > 1, join the states together
            plots.append(PlotArgs(plot_name, states=c))
        for h in args.hidden_curves:
            plots.append(PlotArgs(h, hidden=True))
        plotbox_args = PlotboxArgs(title=args.name, plots=plots, legend=args.legend)

        return plotbox_args
