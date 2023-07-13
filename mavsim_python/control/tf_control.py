"""
tf_control
    - Beard & McLain, PUP, 2012
    - Last Update:
        2/10/2021 - TWM

    Difference equation implementation of continous-time first-order transfer function 
    using trapezoidal rule approximation. Used in yaw-damper implementation.

    Transfer function form:  H(s) = u(s)/y(s) = k * (b1*s + b0)/(a1*s + a0)
"""
import numpy as np


class TFControl:
    def __init__(self, k=0.0, n0=0.0, n1=0.0, d0=0.0, d1=0.0, Ts=0.01, limit=1.0):
        self.k = k
        self.n0 = n0
        self.n1 = n1
        self.d0 = d0
        self.d1 = d1
        self.Ts = Ts
        self.limit = limit
        self.y = 0.0
        self.u = 0.0
        self.y_delay_1 = 0.0
        self.u_delay_1 = 0.0
        # coefficients for difference equation
        self.b0 = - k * (2.0 * n1 - Ts * n0) / (2.0 * d1 + Ts * d0)
        self.b1 = k * (2.0 * n1 + Ts * n0) / (2.0 * d1 + Ts * d0)
        self.a0 = (2.0 * d1 - Ts * d0) / (2.0 * d1 + Ts * d0)

    def update(self, y):
        # calculate transfer function output (u) using difference equation
        u = self.a0 * self.u_delay_1 + self.b1 * y + self.b0 * self.y_delay_1
        # saturate transfer function output at limit
        u_sat = self._saturate(u)
        # update the delayed variables
        self.y_delay_1 = y
        self.u_delay_1 = u_sat
        return u_sat

    def _saturate(self, u):
        # saturate u at +- self.limit
        if u >= self.limit:
            u_sat = self.limit
        elif u <= -self.limit:
            u_sat = -self.limit
        else:
            u_sat = u
        return u_sat