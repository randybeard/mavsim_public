"""
observer
    - Beard & McLain, PUP, 2012
    - Last Update:
        3/4/2019 - RWB
        3/6/2024 - RWB
"""
import numpy as np
from scipy import stats
import parameters.control_parameters as CTRL
import parameters.simulation_parameters as SIM
import parameters.sensor_parameters as SENSOR
import parameters.aerosonde_parameters as MAV
from tools.rotations import euler_to_rotation
from tools.wrap import wrap
from message_types.msg_state import MsgState
from message_types.msg_sensors import MsgSensors
from estimators.filters import AlphaFilter, ExtendedKalmanFilterContinuousDiscrete


class Observer:
    def __init__(self, ts):
        # initialized estimated state message
        ##### TODO #####        
        self.ekf = ExtendedKalmanFilterContinuousDiscrete(
            f=self.f, 
            Q = np.diag([
                (0.)**2,  # pn
                (0.)**2,  # pe
                (0.)**2,  # pd
                (0.)**2,  # u
                (0.)**2,  # v
                (0.)**2,  # w
                (0.)**2,  # phi
                (0.)**2,  # theta
                (0.)**2,  # psi
                (0.)**2,  # bx
                (0.)**2,  # by
                (0.)**2,  # bz
                (0.)**2,  # wn
                (0.)**2,  # we
                ]),
            P0= np.diag([
                0**2,  # pn
                0**2,  # pe
                0**2,  # pd
                0**2,  # u
                0**2,  # v
                0**2,  # w
                np.radians(0)**2,  # phi
                np.radians(0)**2,  # theta
                np.radians(0)**2,  # psi
                np.radians(0)**2,  # bx
                np.radians(0)**2,  # by
                np.radians(0)**2,  # bz
                0**2,  # wn
                0**2,  # we
                ]), 
            xhat0=np.array([[
                MAV.north0,  # pn
                MAV.east0,  # pe
                MAV.down0,  # pd
                MAV.Va0,  # u
                0,  # v
                0,  # w
                0,  # phi
                0,  # theta
                MAV.psi0,  # psi
                0,  # bx
                0,  # by
                0,  # bz
                0,  # wn
                0,  # we
                ]]).T, 
            Qu=np.diag([
                SENSOR.gyro_sigma**2, 
                SENSOR.gyro_sigma**2, 
                SENSOR.gyro_sigma**2, 
                SENSOR.accel_sigma**2,
                SENSOR.accel_sigma**2,
                SENSOR.accel_sigma**2]), 
            Ts=ts,
            N=10
            )
        self.R_analog = np.diag([
            SENSOR.abs_pres_sigma**2,
            SENSOR.diff_pres_sigma**2,
            (0.01)**2
        ])
        self.R_gps = np.diag([
            SENSOR.gps_n_sigma**2,
            SENSOR.gps_e_sigma**2,
            SENSOR.gps_Vg_sigma**2,
            SENSOR.gps_course_sigma**2
        ])
        self.R_pseudo = np.diag([
                    (0.0)**2,  # pseudo measurement #1         ##### TODO #####
                    (0.0)**2,  # pseudo measurement #2
                    ])
        initial_measurements = MsgSensors()
        ##### TODO #####
        self.lpf_gyro_x = AlphaFilter(alpha=0., y0=initial_measurements.gyro_x)
        self.lpf_gyro_y = AlphaFilter(alpha=0., y0=initial_measurements.gyro_y)
        self.lpf_gyro_z = AlphaFilter(alpha=0., y0=initial_measurements.gyro_z)
        self.analog_threshold = stats.chi2.isf(q=0.01, df=3)
        self.pseudo_threshold = stats.chi2.isf(q=0.01, df=2)
        self.gps_n_old = 9999
        self.gps_e_old = 9999
        self.gps_Vg_old = 9999
        self.gps_course_old = 9999
        self.estimated_state = MsgState()
        self.elapsed_time = 0

    def update(self, measurement):
        # system input
        u = np.array([[
            measurement.gyro_x, 
            measurement.gyro_y, 
            measurement.gyro_z,
            measurement.accel_x, 
            measurement.accel_y, 
            measurement.accel_z,
            ]]).T
        xhat, P = self.ekf.propagate_model(u)
        # update with analog measurement
        y_analog = np.array([
            [measurement.abs_pressure],
            [measurement.diff_pressure],
            [0.0], # sideslip pseudo measurement
            ])
        xhat, P = self.ekf.measurement_update(
            y=y_analog, 
            u=u,
            h=self.h_analog,
            R=self.R_analog)
        # update with wind triangle pseudo measurement
        y_pseudo = np.array([
            [0.],
            [0.], 
            ])
        xhat, P = self.ekf.measurement_update(
            y=y_pseudo, 
            u=u,
            h=self.h_pseudo,
            R=self.R_pseudo)
        # only update GPS when one of the signals changes
        if (measurement.gps_n != self.gps_n_old) \
            or (measurement.gps_e != self.gps_e_old) \
            or (measurement.gps_Vg != self.gps_Vg_old) \
            or (measurement.gps_course != self.gps_course_old):
            state = to_MsgState(xhat) 
                # need to do this to get the current chi to wrap meaurement
            y_chi = wrap(measurement.gps_course, state.chi)
            y_gps = np.array([
                [measurement.gps_n], 
                [measurement.gps_e], 
                [measurement.gps_Vg], 
                [y_chi]])
            xhat, P = self.ekf.measurement_update(
                y=y_gps, 
                u=u,
                h=self.h_gps,
                R=self.R_gps)
            # update stored GPS signals
            self.gps_n_old = measurement.gps_n
            self.gps_e_old = measurement.gps_e
            self.gps_Vg_old = measurement.gps_Vg
            self.gps_course_old = measurement.gps_course
        # convert internal xhat to MsgState format
        self.estimated_state = to_MsgState(xhat)
        self.estimated_state.p = self.lpf_gyro_x.update(measurement.gyro_x) \
            - self.estimated_state.bx
        self.estimated_state.q = self.lpf_gyro_y.update(measurement.gyro_y) \
            - self.estimated_state.by
        self.estimated_state.r = self.lpf_gyro_z.update(measurement.gyro_z) \
            - self.estimated_state.bz
        self.elapsed_time += SIM.ts_control
        return self.estimated_state

    def f(self, x:np.ndarray, u:np.ndarray)->np.ndarray:
        # system dynamics for propagation model: xdot = f(x, u)
        ##### TODO #####
        # pos   = x[0:3]
        vel = x[3:6]
        Theta = x[6:9]
        bias = x[9:12]
        # wind = np.array([[x.item(12), x.item(13), 0]]).T
        y_gyro = u[0:3]
        y_accel = u[3:6]
        xdot = np.concatenate((pos_dot, vel_dot, Theta_dot, bias_dot, wind_dot), axis=0)
        return xdot

    def h_analog(self, x:np.ndarray, u:np.ndarray)->np.ndarray:
        ##### TODO #####
        # analog sensor measurements and pseudo measurements
        pos = x[0:3]
        vel_body = x[3:6]
        Theta = x[6:9]
        #bias = x[9:12]
    
        y = np.array([[abs_pres, diff_pres, sideslip]]).T
        return y

    def h_gps(self, x:np.ndarray, u:np.ndarray)->np.ndarray:
        ##### TODO #####        
        # measurement model for gps measurements
        pos = x[0:3]
        vel_body = x[3:6]
        Theta = x[6:9]

        y = np.array([[pn, pe, Vg, chi]]).T
        return y

    def h_pseudo(self, x:np.ndarray, u:np.ndarray)->np.ndarray:
        ##### TODO ##### 
        # measurement model for wind triangale pseudo measurement
        #pos = x[0:3]
        vel_body = x[3:6]
        Theta = x[6:9]
        #bias = x[9:12]
        
        y = np.array([
            [],  # wind triangle x
            [],  # wind triangle y
        ])
        return y


def to_MsgState(x: np.ndarray) -> MsgState:
    state = MsgState()
    state.north = x.item(0)
    state.east = x.item(1)
    state.altitude = -x.item(2)
    vel_body = x[3:6]
    state.phi = x.item(6)
    state.theta = x.item(7)
    state.psi = x.item(8)
    state.bx = x.item(9)
    state.by = x.item(10)
    state.bz = x.item(11)
    state.wn = x.item(12)
    state.we = x.item(13)
    # estimate needed quantities that are not part of state
    R = euler_to_rotation(
        state.phi,
        state.theta,
        state.psi)
    vel_world = R @ vel_body
    wind_world = np.array([[state.wn], [state.we], [0]])
    wind_body = R.T @ wind_world
    vel_rel = vel_body - wind_body
    state.Va = np.linalg.norm(vel_rel)
    state.alpha = np.arctan(vel_rel.item(2) / vel_rel.item(0))
    state.beta = np.arcsin(vel_rel.item(1) / state.Va)
    state.Vg = np.linalg.norm(vel_world)
    state.chi = np.arctan2(vel_world.item(1), vel_world.item(0))
    return state


def cross(vec: np.ndarray)->np.ndarray:
    return np.array([[0, -vec.item(2), vec.item(1)],
                     [vec.item(2), 0, -vec.item(0)],
                     [-vec.item(1), vec.item(0), 0]])


def S(Theta:np.ndarray)->np.ndarray:
    return np.array([[1,
                      np.sin(Theta.item(0)) * np.tan(Theta.item(1)),
                      np.cos(Theta.item(0)) * np.tan(Theta.item(1))],
                     [0,
                      np.cos(Theta.item(0)),
                      -np.sin(Theta.item(0))],
                     [0,
                      (np.sin(Theta.item(0)) / np.cos(Theta.item(1))),
                      (np.cos(Theta.item(0)) / np.cos(Theta.item(1)))]
                     ])
