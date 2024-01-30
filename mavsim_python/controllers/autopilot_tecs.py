"""
autopilot block for mavsim_python - Total Energy Control System
    - Beard & McLain, PUP, 2012
    - Last Update:
        2/14/2020 - RWB
"""
import numpy as np
import parameters.control_parameters as AP
import parameters.aerosonde_parameters as MAV
from tools.transfer_function import TransferFunction
from tools.wrap import wrap
from controllers.pi_control import PIControl
from controllers.pd_control_with_rate import PDControlWithRate
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
        self.yaw_damper = TransferFunction(
                        num=np.array([[AP.yaw_damper_kr, 0]]),
                        den=np.array([[1, AP.yaw_damper_p_wo]]),
                        Ts=ts_control)

        # instantiate TECS controllers
        self.pitch_from_elevator = PDControlWithRate(
                        kp=AP.pitch_kp,
                        kd=AP.pitch_kd,
                        limit=np.radians(45))
        # throttle gains (unitless)
        self.E_kp = 0
        self.E_ki = 0
        # pitch gains
        self.L_kp = 0
        self.L_ki = 0
        # saturated altitude error
        self.h_error_max = 0  # meters
        self.E_integrator = 0
        self.L_integrator = 0
        self.E_error_d1 = 0
        self.L_error_d1 = 0
        self.delta_t_d1 = 0
        self.theta_c_d1 = 0
        self.theta_c_max = 0
        self.Ts = ts_control
        self.commanded_state = MsgState()

    def update(self, cmd, state):
	
	###### TODO ######
        # lateral autopilot


        # longitudinal TECS autopilot


        # construct output and commanded states
        delta = MsgDelta(elevator=0,
                         aileron=0,
                         rudder=0,
                         throttle=0)
        self.commanded_state.altitude = 0                
        self.commanded_state.Va = 0
        self.commanded_state.phi = 0
        self.commanded_state.theta = 0
        self.commanded_state.chi = 0
        return delta, self.commanded_state

    def saturate(self, input, low_limit, up_limit):
        if input <= low_limit:
            output = low_limit
        elif input >= up_limit:
            output = up_limit
        else:
            output = input
        return output
