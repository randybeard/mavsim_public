#!/usr/bin/env python
import pyqtgraph as pg
from collections import OrderedDict
from state_plotter.plotter_args import PlotArgs
from state_plotter.state_data import StateData
import numpy as np
from pdb import set_trace

class StatePlot():
    def __init__(self, plotbox, args):
        if not isinstance(args, PlotArgs):
            raise TypeError('\'args\' argument must be of type PlotArgs')
        # Collect params
        self.name = args.name
        self.connect = args.connect
        self.symbol = args.symbol
        self.symbol_size = args.symbol_size
        self.px_mode = args.px_mode
        self.color = args.color
        self.dimension = len(args.state_names)
        # Initialize the pyqtgraph plot plot
        self.plotbox = plotbox
        # Handle hidden plots
        self.hidden = args.hidden
        if not self.hidden:
            self.plot = self.plotbox.plot(name=self.name)
        # Handle Sigma error bound plots
        self.sigma_bounds = args.sigma_bounds
        self.has_sigma = (self.sigma_bounds is not None)
        self.sigma_plots = {}
        if self.has_sigma:
            dash_size = 4
            space_scale = 1.5
            for bound in self.sigma_bounds:
                space_size = bound*dash_size*space_scale
                dashed_pen = pg.mkPen(self.color, dash=[dash_size, space_size])
                if self.dimension == 1:
                    lower_plot = self.plotbox.plot(name="{} {}-sigma error bound".format(self.name, bound))
                    upper_plot = self.plotbox.plot()
                    self.sigma_plots[bound] = {'lower_plot':lower_plot,
                                               'upper_plot':upper_plot,
                                               'pen':dashed_pen}
                else:
                    sigma_plot = self.plotbox.plot(name="{} {}-sigma error bound".format(self.name, bound))
                    self.sigma_plots[bound] = {'plot':sigma_plot,
                                               'pen':dashed_pen}
        self.marker = None
        self.marker_scale = 0.04 # Percentage of the minimum plotbox dimension for the circle radius
        if self.dimension == 2:
            self.marker = plotbox.plot()
            self.xy_marker_circle = self._get_ellipse((0,0), radius=1.0)

        # Initialize states
        self.states = OrderedDict()

        for name in args.state_names:
            self.add_state(name, self.sigma_bounds, args.max_length, args.is_angle, args.rad2deg)

    def add_state(self, name, sigma_bounds=None, max_length=None, is_angle=False, rad2deg=False):
        self.states[name] = StateData(sigma_bounds=sigma_bounds, max_length=max_length, is_angle=is_angle, rad2deg=rad2deg)

    def get_states(self):
        return self.states

    def update(self):
        if self.hidden:
            return
        # Get data from state objects
        state_objs = list(self.states.values())
        if self.dimension == 1:
            x_data = state_objs[0].get_time_vec()
            y_data = state_objs[0].get_data_vec()
            if self.has_sigma:
                for bound in self.sigma_bounds:
                    pen = self.sigma_plots[bound]['pen']
                    self.sigma_plots[bound]['lower_plot'].setData(x_data, state_objs[0].sigma_data[bound]['lower'], pen=pen)
                    self.sigma_plots[bound]['upper_plot'].setData(x_data, state_objs[0].sigma_data[bound]['upper'], pen=pen)
        elif self.dimension == 2:
            x_data = state_objs[0].get_data_vec()
            y_data = state_objs[1].get_data_vec()
            if self.has_sigma:
                for bound in self.sigma_bounds:
                    pen = self.sigma_plots[bound]['pen']
                    center = [x_data[-1], y_data[-1]]
                    radius = [bound*state_objs[i].get_current_sigma() for i in (0,1)]
                    ellipse = self._get_ellipse(center=center, radius=radius)
                    self.sigma_plots[bound]['plot'].setData(ellipse[0], ellipse[1], pen=pen)
        else:
            raise NotImplementedError('Plots with dimension > 2 are not yet supported.')

        # Update the data for the plot (and marker, if necessary)
        if not self.connect:
            self.plot.setData(x_data, y_data, pen=None, symbol=self.symbol,
                               symbolSize=self.symbol_size, symbolPen=self.color, pxMode=self.px_mode)
        else:
            self.plot.setData(x_data, y_data, pen=self.color)
            # Plot marker
            if self.marker is not None and len(x_data) > 0 and len(y_data) > 0:
                x_range = self.plotbox.vb.targetRange()[0]
                y_range = self.plotbox.vb.targetRange()[1]
                scale = self.marker_scale*min(x_range[1]-x_range[0], y_range[1]-y_range[0])
                marker = scale*self.xy_marker_circle + np.array([[x_data[-1]], [y_data[-1]]])
                self.marker.setData(marker[0], marker[1], pen=self.color)

    def _get_ellipse(self, center, radius):
        N = 100
        theta = np.linspace(0, 2*np.pi, N)
        if isinstance(radius, float) or isinstance(radius, int):
            radius = [radius, radius]
        x = np.cos(theta)*radius[0] + center[0]
        y = np.sin(theta)*radius[1] + center[1]
        return np.array([x,y])
