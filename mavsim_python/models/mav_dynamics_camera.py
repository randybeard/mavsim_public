"""
mavDynamics 
    - this file implements the dynamic equations of motion for MAV
    - use unit quaternion for the attitude state
    
mavsim_python
    - Beard & McLain, PUP, 2012
    - Update history:  
        2/24/2020 - RWB
"""
import numpy as np

# load message types
from message_types.msg_state import MsgState
from message_types.msg_sensors import MsgSensors
from message_types.msg_delta import MsgDelta
import parameters.camera_parameters as CAM

import parameters.aerosonde_parameters as MAV
import parameters.sensor_parameters as SENSOR
from tools.rotations import Quaternion2Rotation, Quaternion2Euler, Euler2Rotation

class MavDynamics:
    def __init__(self, Ts):
        self._ts_simulation = Ts
        # set initial states based on parameter file
        # _state is the 13x1 internal state of the aircraft that is being propagated:
        # _state = [pn, pe, pd, u, v, w, e0, e1, e2, e3, p, q, r]
        # We will also need a variety of other elements that are functions of the _state and the wind.
        # self.true_state is a 19x1 vector that is estimated and used by the autopilot to control the aircraft:
        # true_state = [pn, pe, h, Va, alpha, beta, phi, theta, chi, p, q, r, Vg, wn, we, psi, gyro_bx, gyro_by, gyro_bz]
        self._state = np.array([[MAV.north0],  # (0)
                               [MAV.east0],   # (1)
                               [MAV.down0],   # (2)
                               [MAV.u0],    # (3)
                               [MAV.v0],    # (4)
                               [MAV.w0],    # (5)
                               [MAV.e0],    # (6)
                               [MAV.e1],    # (7)
                               [MAV.e2],    # (8)
                               [MAV.e3],    # (9)
                               [MAV.p0],    # (10)
                               [MAV.q0],    # (11)
                               [MAV.r0],    # (12)
                               [CAM.az0],   # (13)
                               [CAM.el0],   # (14)
                               ])
        # store wind data for fast recall since it is used at various points in simulation
        self._wind = np.array([[0.], [0.], [0.]])  # wind in NED frame in meters/sec
        # store forces to avoid recalculation in the sensors function
        self._forces = np.array([[0.], [0.], [0.]])
        self._Va = MAV.u0
        self._alpha = 0
        self._beta = 0
        # initialize true_state message
        self.true_state = MsgState()
        # initialize the sensors message
        self._sensors = MsgSensors()
        # random walk parameters for GPS
        self._gps_eta_n = 0.
        self._gps_eta_e = 0.
        self._gps_eta_h = 0.
        # timer so that gps only updates every ts_gps seconds
        self._t_gps = 999.  # large value ensures gps updates at initial time.
        # update velocity data and forces and moments
        self._update_velocity_data()
        self._forces_moments(delta=MsgDelta())


    ###################################
    # public functions
    def update(self, delta, wind):
        '''
            Integrate the differential equations defining dynamics, update sensors
            delta = (delta_a, delta_e, delta_r, delta_t) are the control inputs
            wind is the wind vector in inertial coordinates
            Ts is the time step between function calls.
        '''
        # gimbal azimuth dynamics
        if ( not ((abs(self._state[13,0]) >= CAM.az_limit) and (delta.gimbal_az > 0) )
            and not ((abs(self._state[13,0])<= - CAM.az_limit) and (delta.gimbal_az<0))):
                self._state[13, 0] += self._ts_simulation * CAM.az_gain * delta.gimbal_az
        # gimbal elevation dynamics
        if ( not ((abs(self._state[14,0]) >= CAM.el_limit) and (delta.gimbal_el > 0) )
            and not ((abs(self._state[14,0])<= - CAM.el_limit) and (delta.gimbal_el<0))):
                self._state[14, 0] += self._ts_simulation * CAM.el_gain * delta.gimbal_el
                
        # get forces and moments acting on rigid bod
        forces_moments = self._forces_moments(delta)

        # Integrate ODE using Runge-Kutta RK4 algorithm
        time_step = self._ts_simulation
        k1 = self._derivatives(self._state[0:13], forces_moments)
        k2 = self._derivatives(self._state[0:13] + time_step/2.*k1, forces_moments)
        k3 = self._derivatives(self._state[0:13] + time_step/2.*k2, forces_moments)
        k4 = self._derivatives(self._state[0:13] + time_step*k3, forces_moments)
        self._state[0:13] += time_step/6 * (k1 + 2*k2 + 2*k3 + k4)

        # normalize the quaternion
        e0 = self._state.item(6)
        e1 = self._state.item(7)
        e2 = self._state.item(8)
        e3 = self._state.item(9)
        normE = np.sqrt(e0**2+e1**2+e2**2+e3**2)
        self._state[6][0] = self._state.item(6)/normE
        self._state[7][0] = self._state.item(7)/normE
        self._state[8][0] = self._state.item(8)/normE
        self._state[9][0] = self._state.item(9)/normE

        # update the airspeed, angle of attack, and side slip angles using new state
        self._update_velocity_data(wind)
        # update the message class for the true state
        self._update_true_state()

    def sensors(self):
        "Return value of sensors on MAV: gyros, accels, absolute_pressure, dynamic_pressure, GPS"
       
        # simulate rate gyros(units are rad / sec)
        self._sensors.gyro_x = 0
        self._sensors.gyro_y = 0
        self._sensors.gyro_z = 0

        # simulate accelerometers(units of g)
        self._sensors.accel_x = 0
        self._sensors.accel_y = 0
        self._sensors.accel_z = 0

        # simulate magnetometers
        # magnetic field in provo has magnetic declination of 12.5 degrees
        # and magnetic inclination of 66 degrees
        self._sensors.mag_x = 0
        self._sensors.mag_y = 0
        self._sensors.mag_z = 0

        # simulate pressure sensors
        self._sensors.abs_pressure = 0
        self._sensors.diff_pressure = 0
        
        # simulate GPS sensor
        if self._t_gps >= SENSOR.ts_gps:
            self._gps_eta_n = 0
            self._gps_eta_e = 0
            self._gps_eta_h = 0
            self._sensors.gps_n = 0
            self._sensors.gps_e = 0
            self._sensors.gps_h = 0
            self._sensors.gps_Vg = 0
            self._sensors.gps_course = 0
            self._t_gps = 0.
        else:
            self._t_gps += self._ts_simulation
        return self._sensors

    def external_set_state(self, new_state):
        self._state = new_state

    ###################################
    # private functions
    def _derivatives(self, state, forces_moments):
        """
        for the dynamics xdot = f(x, u), returns f(x, u)
        """
        ##### TODO #####
        #  -- Copy From mav_dynamic_control.py --

        x_dot = np.array([[0,0,0,0,0,0,0,0,0,0,0,0,0]]).T
        return x_dot

    def _update_velocity_data(self, wind=np.zeros((6,1))):
        steady_state = wind[0:3]
        gust = wind[3:6]

        ##### TODO #####
        #  -- Copy From mav_dynamic_control.py --
        # convert wind vector from world to body frame (self._wind = ?)

        # velocity vector relative to the airmass ([ur , vr, wr]= ?)

        # compute airspeed (self._Va = ?)

        # compute angle of attack (self._alpha = ?)
        
        # compute sideslip angle (self._beta = ?)

    def _forces_moments(self, delta):
        """
        return the forces on the UAV based on the state, wind, and control surfaces
        :param delta: np.matrix(delta_a, delta_e, delta_r, delta_t)
        :return: Forces and Moments on the UAV np.matrix(Fx, Fy, Fz, Ml, Mn, Mm)
        """
        ##### TODO ###### 
        # #  -- Copy From mav_dynamic_control.py --
        # extract states (phi, theta, psi, p, q, r)

        # compute gravitaional forces ([fg_x, fg_y, fg_z])

        # compute Lift and Drag coefficients (CL, CD)

        # compute Lift and Drag Forces (F_lift, F_drag)

        # propeller thrust and torque
        # thrust_prop, torque_prop = self._motor_thrust_torque(self._Va, delta.throttle)

        # compute longitudinal forces in body frame (fx, fz)

        # compute lateral forces in body frame (fy)

        # compute logitudinal torque in body frame (My)

        # compute lateral torques in body frame (Mx, Mz)

        forces_moments = np.array([[0, 0, 0, 0, 0, 0]]).T
        return forces_moments

    def _motor_thrust_torque(self, Va, delta_t):
        # compute thrust and torque due to propeller
        ##### TODO #####
        # #  -- Copy From mav_dynamic_control.py --

        # map delta_t throttle command(0 to 1) into motor input voltage
        # v_in =

        # Angular speed of propeller (omega_p = ?)

        # thrust and torque due to propeller
        thrust_prop = 0
        torque_prop = 0

        return thrust_prop, torque_prop

    def _update_true_state(self):
        # update the class structure for the true state:
        #   [pn, pe, h, Va, alpha, beta, phi, theta, chi, p, q, r, Vg, wn, we, psi, gyro_bx, gyro_by, gyro_bz]
        phi, theta, psi = Quaternion2Euler(self._state[6:10])
        pdot = Quaternion2Rotation(self._state[6:10]) @ self._state[3:6]
        self.true_state.north = self._state.item(0)
        self.true_state.east = self._state.item(1)
        self.true_state.altitude = -self._state.item(2)
        self.true_state.Va = self._Va
        self.true_state.alpha = self._alpha
        self.true_state.beta = self._beta
        self.true_state.phi = phi
        self.true_state.theta = theta
        self.true_state.psi = psi
        self.true_state.Vg = np.linalg.norm(pdot)
        self.true_state.gamma = np.arcsin(pdot.item(2) / self.true_state.Vg)
        self.true_state.chi = np.arctan2(pdot.item(1), pdot.item(0))
        self.true_state.p = self._state.item(10)
        self.true_state.q = self._state.item(11)
        self.true_state.r = self._state.item(12)
        self.true_state.wn = self._wind.item(0)
        self.true_state.we = self._wind.item(1)
        self.true_state.bx = SENSOR.gyro_x_bias
        self.true_state.by = SENSOR.gyro_y_bias
        self.true_state.bz = SENSOR.gyro_z_bias
        self.true_state.camera_az = self._state.item(13)
        self.true_state.camera_el = self._state.item(14)
