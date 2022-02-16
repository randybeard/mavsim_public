"""
msg_delta
    - messages type for input to the aircraft
    
part of mavsim_python
    - Beard & McLain, PUP, 2012
    - Last update:
        2/27/2020 - RWB
"""
import numpy as np


class MsgDelta:
    def __init__(self,
                 elevator=0.0,
                 aileron=0.0,
                 rudder=0.0,
                 throttle=0.5):
        self.elevator = elevator  # elevator command
        self.aileron = aileron  # aileron command
        self.rudder = rudder  # rudder command
        self.throttle = throttle  # throttle command

    def to_array(self):
        return np.array([[self.elevator],
                         [self.aileron],
                         [self.rudder],
                         [self.throttle]])

    def from_array(self, u):
        self.elevator = u.item(0)
        self.aileron = u.item(1)
        self.rudder = u.item(2)
        self.throttle = u.item(3)

    def print(self):
        print('elevator=', self.elevator,
              'aileron=', self.aileron,
              'rudder=', self.rudder,
              'throttle=', self.throttle)

