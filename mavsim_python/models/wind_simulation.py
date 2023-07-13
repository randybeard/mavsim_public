"""
Class to determine wind velocity at any given moment,
calculates a steady wind speed and uses a stochastic
process to represent wind gusts. (Follows section 4.4 in uav book)
"""
from tools.transfer_function import TransferFunction
import numpy as np


class WindSimulation:
    def __init__(self, Ts, gust_flag = True, steady_state = np.array([[0., 0., 0.]]).T):
        # steady state wind defined in the inertial frame
        self._steady_state = steady_state
        ##### TODO #####

        #   Dryden gust model parameters (pg 56 UAV book)

        # Dryden transfer functions (section 4.4 UAV book) - Fill in proper num and den
        self.u_w = TransferFunction(num=np.array([[0]]), den=np.array([[1,1]]),Ts=Ts)
        self.v_w = TransferFunction(num=np.array([[0,0]]), den=np.array([[1,1,1]]),Ts=Ts)
        self.w_w = TransferFunction(num=np.array([[0,0]]), den=np.array([[1,1,1]]),Ts=Ts)
        self._Ts = Ts

    def update(self):
        # returns a six vector.
        #   The first three elements are the steady state wind in the inertial frame
        #   The second three elements are the gust in the body frame
        gust = np.array([[self.u_w.update(np.random.randn())],
                         [self.v_w.update(np.random.randn())],
                         [self.w_w.update(np.random.randn())]])
        return np.concatenate(( self._steady_state, gust ))

