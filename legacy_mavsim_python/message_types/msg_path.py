"""
msg_path
    - messages type for input to path follower
    
part of mavsim_python
    - Beard & McLain, PUP, 2012
    - Last update:
        3/11/2019 - RWB
"""
import numpy as np


class MsgPath:
    def __init__(self):
        # type='line' means straight line following, type='orbit' means orbit following
        self.type = 'line'
        #self.type = 'orbit'
        # desired airspeed along the path
        self.airspeed = 25
        # origin of the straight path line (r)
        self.line_origin = np.array([[0.0, 0.0, 0.0]]).T
        # direction of line -unit vector- (q)
        self.line_direction = np.array([[1.0, 0.0, 0.0]]).T
        # center of the orbit (c)
        self.orbit_center = np.array([[0.0, 0.0, 0.0]]).T
        # radius of the orbit (rho)
        self.orbit_radius = 50
        # orbit direction: 'CW'==clockwise, 'CCW'==counter clockwise
        self.orbit_direction = 'CW'
        # flag that indicates that path has been plotted
        self.plot_updated = False
