"""
mavsimPy
    Homework check for chapter 3
"""
import sys
sys.path.append('..')
import numpy as np
from models.mav_dynamics_forces import MavDynamics
import parameters.simulation_parameters as SIM
import parameters.aerosonde_parameters as MAV


state = np.array([[5], [2], [-20], [5],
                        [0], [0], [1], [0], [0],
                        [0], [1], [0.5], [0], [0], [0]])
forces_moments = np.array([[10, 5, 0, 0, 14, 0]]).T
mav = MavDynamics(SIM.ts_simulation)
x_dot = mav._derivatives(state, forces_moments)
print("x_dot case 1: ", x_dot.transpose())
print(" ")

#### expected output #####
# x_dot case 1:  [[ 5, 0, 0, 0.90909091, 0.45454545, 2.5, -0,
#                   0.5, 0.25, 0, 0.06073576, 12.22872247, -0.08413156]]

state = np.array([[5], [2], [-20], [0],
                        [3], [6], [1], [.6], [0],
                        [.2], [0], [0], [3], [0], [0]])
forces_moments = np.array([[10, 5, 0, 0, 14, 0]]).T
mav = MavDynamics(SIM.ts_simulation)
x_dot = mav._derivatives(state, forces_moments)
print("x_dot case 2: ", x_dot.transpose())

#### expected output #####
# x_dot case 2:  [[ 0.08746356, -1.96793003, 2.79883382, 9.90909091, 0.45454545, 0.
                    # -0.3, 0, -0.9, 1.5, 0, 13.28951542, 0. ]]


