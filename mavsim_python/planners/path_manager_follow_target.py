"""
mavsim_python: drawing tools
    - Beard & McLain, PUP, 2012
    - Update history:
        4/15/2019 - RWB
        3/30/2022 - RWB
        4/1/2022 - RWB
"""

import numpy as np
from message_types.msg_path import MsgPath


class PathManager:
    def __init__(self):
        # message sent to path follower
        self.path = MsgPath()
        self.manager_requests_waypoints = True

    def update(self, target_position):
        self.path.type = 'orbit'
        self.path.airspeed = 25
        self.path.orbit_center[0, 0] = target_position.item(0)
        self.path.orbit_center[1, 0] = target_position.item(1)
        self.path.orbit_center[2, 0] = -200
        self.path.orbit_radius = 150
        self.path.orbit_direction = 'CW'
        return self.path
