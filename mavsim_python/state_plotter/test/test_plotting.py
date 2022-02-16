#!/usr/bin/env python
import time
from builtins import input
from IPython.core.debugger import set_trace

import numpy as np

from state_plotter.Plotter import Plotter
from state_plotter.plotter_args import *

plotter = Plotter(plotting_frequency=1)

### Define plot names

## Simple string definitions
first_row = ['x', 'y', 'z']

## Multiple plots in a plotbox (using PlotboxArgs)
# -Simply add multiple plot names
phi_plots = PlotboxArgs(plots=['phi', 'phi_e'])
# -Add title to the plotbox
theta_plots = PlotboxArgs(
    title="Multiple theta plots",
    plots=['theta', 'theta_e']
)
# -Use Plot args to name different curves in the legend,
# -Use 'labels' to get more detailed x and y labels
# -Use rad2deg to automatically wrap angles and convert radians to degrees
psi_plots = PlotboxArgs(
    title="Multiple psi plots",
    plots=[PlotArgs('True psi', states=['psi']),
           PlotArgs('Estimated psi', states='psi_e')],
    labels={'left':'Psi (deg)', 'bottom':'Time (s)'},
    rad2deg=True
)
second_row = [phi_plots, theta_plots, psi_plots]

## Two dimensional plots
# -Simple 2D plot. Use PlotArgs to combine two states into a 2D plot
xy_plot = PlotboxArgs(
    title="XY Plane",
    plots=[PlotArgs(states=['x', 'y']),
           PlotArgs(states=['x_truth', 'y_truth'])]
)
# -Add names to the different 2D curves
xz_plot = PlotboxArgs(
    title="XZ plane",
    plots=[PlotArgs('Estimated xz position', states=['x', 'z']),
           PlotArgs('True xz position', states=['x_truth', 'z_truth'])]
)
# -Add extra labels to the plotbox for clarity
# -Use max_length to only plot the last 100 data points
yz_plot = PlotboxArgs(
    title="YZ plane",
    plots=[PlotArgs('Estimated yz position', states=['y', 'z']),
           PlotArgs('True yz position', states=['y_truth', 'z_truth'])],
    labels={'left':'Y Position (m)', 'bottom':'Z Position (m)'},
    max_length=100
)
third_row = [xy_plot, xz_plot, yz_plot]

# Use a list of lists to specify plotting window structure (3 rows, each with 3 plots)
plots = [first_row,
         second_row,
         third_row
        ]

# Add plots to the window
plotter.add_plotboxes(plots)

# Define and label vectors for more convenient/natural data input
plotter.define_input_vector('position', ['x', 'y', 'z'])
plotter.define_input_vector('true_position', ['x_truth', 'y_truth', 'z_truth'])
plotter.define_input_vector('attitude', ['phi', 'theta', 'psi'])
plotter.define_input_vector('estimated_attitude', ['phi_e', 'theta_e', 'psi_e'])


# setup simulation timing
T = 5
Ts = 0.01
tvec = np.linspace(0, T, num=int((1/Ts)*T))

# run the simulation
for idx, t in enumerate(tvec):
    # Make some sinusoids and stuff
    x = np.sin(2*np.pi*1*t)
    y = np.cos(2*np.pi*0.5*t)
    z = t + np.cos(2*np.pi*2*t)

    x_t = 1.5*np.sin(2*np.pi*1*t)
    y_t = 1.5*np.cos(2*np.pi*0.5*t)
    z_t = t

    phi = np.sin(2*np.pi*1*t)
    theta = np.cos(2*np.pi*0.5*t)
    psi = t

    phi_e = 1.5*np.sin(2*np.pi*1*t)
    theta_e = 1.5*np.cos(2*np.pi*0.5*t)
    psi_e = t + 0.1*np.cos(2*np.pi*2*t)

    ## Add the state data in vectors
    plotter.add_vector_measurement('position', [x, y, z], t)
    plotter.add_vector_measurement('true_position', [x_t, y_t, z_t], t)
    plotter.add_vector_measurement('attitude', [phi, theta, psi], t)
    # Demonstrate plotting with independent measurement intervals
    if np.mod(idx, 3) == 0:
        plotter.add_vector_measurement('estimated_attitude', [phi_e, theta_e, psi_e], t)

    # Update and display the plot
    plotter.update_plots()

# Wait so that the plot doesn't disappear
input("Press any key to end...")
