"""
Class to determine wind velocity at any given moment,
calculates a steady wind speed and uses a stochastic
process to represent wind gusts. (Follows section 4.4 in uav book)
"""
import sys
sys.path.append('..')
from tools.transfer_function import transferFunction
import numpy as np


class WindSimulation:
    def __init__(self, Ts):
        # steady state wind defined in the inertial frame
        self._steady_state = np.array([[0., 0., 0.]]).T
        #self._steady_state = np.array([[0., 5., 0.]]).T

        #   Dryden gust model parameters (section 4.4 UAV book)
        Va =  # must set Va to a constant value
        Lu = 
        Lv = 
        Lw = 
        gust_flag = True
        if gust_flag==True:
            sigma_u = 
            sigma_v = 
            sigma_w = 
        else:
            sigma_u = 
            sigma_v = 
            sigma_w = 

        # Dryden transfer functions (section 4.4 UAV book)
        self.u_w = #transferFunction(num=np.array([[___]]), den=np.array([[___]]),Ts=__)
        self.v_w = #transferFunction(num=np.array([[___]]), den=np.array([[___]]),Ts=__)
        self.w_w = #transferFunction(num=np.array([[___]]), den=np.array([[___]]),Ts=__)
        self._Ts = Ts

    def update(self):
        # returns a six vector.
        #   The first three elements are the steady state wind in the inertial frame
        #   The second three elements are the gust in the body frame
        gust = np.array([[self.u_w.update(np.random.randn())],
                         [self.v_w.update(np.random.randn())],
                         [self.w_w.update(np.random.randn())]])
        #gust = np.array([[0.],[0.],[0.]])
        return np.concatenate(( self._steady_state, gust ))

