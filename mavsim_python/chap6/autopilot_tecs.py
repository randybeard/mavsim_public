"""
autopilot block for mavsim_python - Total Energy Control System
    - Beard & McLain, PUP, 2012
    - Last Update:
        2/14/2020 - RWB
"""
import sys
import numpy as np
sys.path.append('..')
import parameters.control_parameters as AP
import parameters.aerosonde_parameters as MAV
from tools.transfer_function import transferFunction
from tools.wrap import wrap
from chap6.pi_control import PIControl
from chap6.pd_control_with_rate import PDControlWithRate
from message_types.msg_state import MsgState
from message_types.msg_delta import MsgDelta


class Autopilot:
    def __init__(self, ts_control):
        # instantiate lateral controllers
        self.roll_from_aileron = PDControlWithRate(
                        kp=AP.roll_kp,
                        kd=AP.roll_kd,
                        limit=np.radians(45))
        self.course_from_roll = PIControl(
                        kp=AP.course_kp,
                        ki=AP.course_ki,
                        Ts=ts_control,
                        limit=np.radians(30))
        self.yaw_damper = transferFunction(
                        num=np.array([[AP.yaw_damper_kr, 0]]),
                        den=np.array([[1, AP.yaw_damper_p_wo]]),
                        Ts=ts_control)

        # instantiate TECS controllers
        self.pitch_from_elevator = PDControlWithRate(
                        kp=AP.pitch_kp,
                        kd=AP.pitch_kd,
                        limit=np.radians(45))
        # throttle gains (unitless)
        self.E_kp = 
        self.E_ki = 
        # pitch gains
        self.L_kp = 
        self.L_ki = 
        # saturated altitude error
        self.h_error_max =   # meters
        self.E_integrator = 
        self.L_integrator = 
        self.E_error_d1 = 
        self.L_error_d1 = 
        self.delta_t_d1 = 
        self.theta_c_d1 = 
        self.theta_c_max = 
        self.Ts = ts_control
        self.commanded_state = MsgState()

    def update(self, cmd, state):

        # lateral autopilot
        chi_c = 
        phi_c = 
        delta_a = 
        delta_r = 

        # longitudinal TECS autopilot
        # error in kinetic energy
        K_error = 
        K_ref = 

        # (saturated) error in potential energy
        U_error = 

        # (normalized) error in total energy and energy difference
        E_error =
        L_error = 

        #  update the integrator(with anti - windup)
        if ():
            self.E_integrator = 

        if ():
            self.L_integrator = 

        delta_t = 
        theta_c = 
        delta_e = 
        self.E_error_d1 = E_error
        self.L_error_d1 = L_error
        self.delta_t_d1 = delta_t
        self.theta_c_d1 = theta_c

        # construct output and commanded states
        delta = MsgDelta(elevator=delta_e,
                         aileron=delta_a,
                         rudder=delta_r,
                         throttle=delta_t)
        self.commanded_state.h = 
        self.commanded_state.Va = 
        self.commanded_state.phi = 
        self.commanded_state.theta = 
        self.commanded_state.chi = 
        return delta, self.commanded_state

    def saturate(self, input, low_limit, up_limit):
        if input <= low_limit:
            output = low_limit
        elif input >= up_limit:
            output = up_limit
        else:
            output = input
        return output
