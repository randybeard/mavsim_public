"""
observer
    - Beard & McLain, PUP, 2012
    - Last Update:
        3/4/2019 - RWB
"""
import sys
import numpy as np
from scipy import stats
sys.path.append('..')
import parameters.control_parameters as CTRL
import parameters.simulation_parameters as SIM
import parameters.sensor_parameters as SENSOR
import parameters.aerosonde_parameters as MAV
from tools.rotations import Euler2Rotation
from tools.wrap import wrap

from message_types.msg_state import MsgState

class Observer:
    def __init__(self, ts_control):
        # initialized estimated state message
        self.ekf = ekfFullState()

    def update(self, measurement):
        estimated_state = self.ekf.update(measurement)
        return estimated_state

class ekfFullState:
    # implement continous-discrete EKF to estimate full state
    def __init__(self):
        self.Q = 
        self.Q_gyro = 
        self.Q_accel = 
        self.R_analog = 
        self.R_gps = 
        self.N =   # number of prediction step per sample
        self.Ts = (SIM.ts_control / self.N)
        self.xhat = 
        self.P = 
        self.analog_threshold = #stats.chi2.isf()
        self.gps_n_old = 9999
        self.gps_e_old = 9999
        self.gps_Vg_old = 9999
        self.gps_course_old = 9999
        self.estimated_state = MsgState()

    def update(self, measurement):
        self.propagate_model(measurement)
        self.measurement_update(measurement)
        # write out estimate state
        self.estimated_state.pn = self.xhat.item(0)
        self.estimated_state.pe = self.xhat.item(1)
        self.estimated_state.h = -self.xhat.item(2)
        self.estimated_state.u = self.xhat.item(3)
        self.estimated_state.v = self.xhat.item(4)
        self.estimated_state.w = self.xhat.item(5)
        self.estimated_state.phi = self.xhat.item(6)
        self.estimated_state.theta = self.xhat.item(7)
        self.estimated_state.psi = self.xhat.item(8)
        self.estimated_state.bx = self.xhat.item(9)
        self.estimated_state.by = self.xhat.item(10)
        self.estimated_state.bz = self.xhat.item(11)
        self.estimated_state.wn = self.xhat.item(12)
        self.estimated_state.we = self.xhat.item(13)
        # estimate needed quantities that are not part of state
        R = Euler2Rotation(self.estimated_state.phi,
                           self.estimated_state.theta,
                           self.estimated_state.psi)
        vel_body = self.xhat[3:6]
        vel_world =
        wind_world = 
        wind_body = 
        self.estimated_state.Va = 
        self.estimated_state.alpha = 
        self.estimated_state.beta = 
        self.estimated_state.Vg = 
        self.estimated_state.chi = 
        self.estimated_state.p = 
        self.estimated_state.q = 
        self.estimated_state.r = 
        return self.estimated_state

    def propagate_model(self, measurement):
        # model propagation
        for i in range(0, self.N):

            S = 
            vel_cross = 
            # propagate model
            self.xhat = 
            # compute Jacobian
            A = 
            # convert to discrete time models
            A_d = 
            Gg_d =  
            Ga_d = 
            # update P with discrete time model
            self.P = 

    def f(self, x, measurement):
        # system dynamics for propagation model: xdot = f(x, u)
        pos_dot = 
        vel_dot = 
        Theta_dot = 
        bias_dot = 
        wind_dot = 
        f_ = np.concatenate((pos_dot, vel_dot, Theta_dot, bias_dot, wind_dot), axis=0)
        return f_

    def measurement_update(self, measurement):
        # always update based on sensor measurements
        h = self.h_analog(self.xhat, measurement)
        C = jacobian(self.h_analog, self.xhat, measurement)
        y = np.array([[measurement.abs_pressure,
                      measurement.diff_pressure,
                      0.0
                      ]]).T
        S_inv = 
        if False:  #(y - h).T @ S_inv @ (y - h) < self.analog_threshold:
            L = 
            self.P = 
            self.xhat = 

        # only update GPS when one of the signals changes
        if (measurement.gps_n != self.gps_n_old) \
            or (measurement.gps_e != self.gps_e_old) \
            or (measurement.gps_Vg != self.gps_Vg_old) \
            or (measurement.gps_course != self.gps_course_old):

            h = #self.h_gps()
            C = #jacobian()
            y_chi = #wrap()
            y = np.array([[measurement.gps_n, measurement.gps_e, measurement.gps_Vg, y_chi]]).T
            if np.linalg.norm(y-h)<0.5:
                L = 
                self.P = 
                self.xhat = 

            # update stored GPS signals
            self.gps_n_old = measurement.gps_n
            self.gps_e_old = measurement.gps_e
            self.gps_Vg_old = measurement.gps_Vg
            self.gps_course_old = measurement.gps_course

    def h_analog(self, x, measurement):
        # analog sensor measurements and pseudo measurements
        abs_pres = 
        diff_pres = 
        sideslip = 
        h = np.array([[abs_pres, diff_pres, sideslip]]).T
        return h

    def h_gps(self, x, measurement):
        # measurement model for gps measurements
        north = 
        east = 
        Vg = 
        chi = 
        h = np.array([[north, east, Vg, chi]]).T
        return h

def jacobian(fun, x, measurement):
    # compute jacobian of fun with respect to x
    f = fun(x, measurement)
    m = f.shape[0]
    n = x.shape[0]
    eps = 0.01  # deviation
    J = np.zeros((m, n))
    for i in range(0, n):
        x_eps = np.copy(x)
        x_eps[i][0] += eps
        f_eps = fun(x_eps, measurement)
        df = (f_eps - f) / eps
        J[:, i] = df[:, 0]
    return J