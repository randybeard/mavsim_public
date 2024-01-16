"""
target geolocation algorithm
    - Beard & McLain, PUP, 2012
    - Updated:
        4/1/2022 - RWB
        4/6/2022 - RWB
"""
import numpy as np
from scipy import stats
import parameters.simulation_parameters as SIM
import parameters.camera_parameters as CAM
from tools.rotations import Euler2Rotation


class Geolocation:
    def __init__(self, ts_control):
        # initialize EKF for geolocation

        ###### TODO ######
        self.xhat = np.array([
            [0.],  # north position
            [0.],  # east position
            [0.],  # down position
            [0.],  # north velocity
            [0.],  # east velocity
            [0.],  # down velocity
            [0.], # distance to target L
            ])
        self.Q = np.diag([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.R = np.diag([0.0, 0.0, 0.0, 0.0])
        self.N = 1  # number of prediction step per sample
        self.P = np.diag([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.gate_threshold = 0 #stats.chi2.isf(q=0, df=0)
        self.Ts = SIM.ts_control/self.N
        # initialize viewer for geolocation error

    def update(self, mav_state, pixels):
        self.propagate_model(mav_state)
        self.measurement_update(mav_state, pixels)
        return self.xhat[0:3, :]  # return estimated NED position

    def propagate_model(self, mav_state):
        # model propagation

        ##### TODO ######
        
        # self.xhat = 
            
        # compute Jacobian
            
        # convert to discrete time models
            
        # update P with discrete time model
        # self.P = ?
        pass
            
    def measurement_update(self, mav_state, pixels):
        # measurement updates
        h = self.h(self.xhat, mav_state)
        C = jacobian(self.h, self.xhat, mav_state)
        y = self.measurements(mav_state, pixels)
        ###### TODO ######
        # self.P = ?
        # self.xhat = ?
        pass

    def f(self, xhat, mav):
        target_position = xhat[0:3]
        target_velocity = xhat[3:6]

        ######  TODO  ######
        # system dynamics for propagation model: xdot = f(x, u)
        f_ = np.zeros((7, 1))
        # position dot = velocity
        
        # velocity dot = zero

        # Ldot
        return f_

    def h(self, xhat, mav):
        ###### TODO ######
        # measurement model y
        h_ = np.array([
            [0],
            [0],
            [0],
            [0],
        ])
        return h_

    def measurements(self, mav, pixels):
        ####### TODO ########
        y = np.zeros((4,1))
        return y

def jacobian(fun, x, mav_state):
    # compute jacobian of fun with respect to x
    f = fun(x, mav_state)
    m = f.shape[0]
    n = x.shape[0]
    eps = 0.0001  # deviation
    J = np.zeros((m, n))
    for i in range(0, n):
        x_eps = np.copy(x)
        x_eps[i][0] += eps
        f_eps = fun(x_eps, mav_state)
        df = (f_eps - f) / eps
        J[:, i] = df[:, 0]
    return J
