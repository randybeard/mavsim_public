"""
target_dynamics
    - this file implements the dynamic equations of motion for ground target moving along city streets
part of mavsim
    - Beard & McLain, PUP, 2012
    - Update history:  
        3/30/2022 - RWB
"""
import numpy as np

# load message types
from message_types.msg_state import MsgState
import parameters.aerosonde_parameters as MAV
from tools.rotations import Quaternion2Euler, Quaternion2Rotation


class TargetDynamics:
    def __init__(self, Ts, map):
        self.velocity = 15 #40.  # nominal velocity of the target
        self.ts_simulation = Ts
        self.map = map
        # randomly select initial position of target
        block_n = np.random.randint(1, map.num_city_blocks - 1)
        block_e = np.random.randint(1, map.num_city_blocks - 1)
        pn0 = block_n * (map.street_width + map.building_width)
        pe0 = block_e * (map.street_width + map.building_width)
        # randomly select initial velocity of target
        temp = np.random.rand()
        vn0 = 0.
        ve0 = 0.
        if temp < 0.25:
            vn0 = self.velocity
        elif temp < 0.5:
            ve0 = self.velocity
        elif temp < 0.75:
            vn0 = -self.velocity
        else:
            ve0 = -self.velocity
        # set initial states based on parameter file
        # _state is the 6x1 internal state of the target
        # _state = [pn, pe, pd, vn, ve, vd] - inertial positions and velocities
        self._state = np.array([[pn0],   # (0) north position
                                [pe0],   # (1) east position
                                [0],   # (2) down position
                                [vn0],    # (3) north velocity
                                [ve0],    # (4) east velocity
                                [0],    # (5) down velocity
                               ])
        self._last_intersection = self.position()

    ###################################
    # public functions
    def update(self):
        # propagate target position
        self._state[0][0] += self.ts_simulation * self._state[3][0]
        self._state[1][0] += self.ts_simulation * self._state[4][0]

        # update target velocity if at an intersection
        if np.linalg.norm(self.position()-self._last_intersection) >= self.map.building_width + self.map.street_width:
            temp = np.random.rand()
            if (temp < 0.25) and (self._state[0][0] < self.map.city_width - self.map.street_width):
                self._state[3][0] = self.velocity
                self._state[4][0] = 0.
            elif (temp < 0.5) and (self._state[1][0] < self.map.city_width - self.map.street_width):
                self._state[3][0] = 0.
                self._state[4][0] = self.velocity
            elif (temp < 0.75) and (self._state[0][0] > self.map.street_width):
                self._state[3][0] = -self.velocity
                self._state[4][0] = 0.
            elif self._state[1][0] > self.map.street_width :
                self._state[3][0] = 0.
                self._state[4][0] = -self.velocity
            else:
                self._state[3][0] = 0.
                self._state[4][0] = self.velocity
            self._last_intersection = self.position()

    # return the inertial position of the target
    def position(self):
        return np.array([[self._state.item(0)], [self._state.item(1)], [self._state.item(2)]])

    # return the inertial velocity of the target
    def velocity(self):
        return np.array([[self._state.item(3)], [self._state.item(4)], [self._state.item(5)]])

