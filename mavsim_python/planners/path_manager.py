"""
mavsim_python: drawing tools
    - Beard & McLain, PUP, 2012
    - Update history:
        4/15/2019 - RWB
        3/30/2022 - RWB
"""

import numpy as np
from planning.dubins_parameters import DubinsParameters
from message_types.msg_path import MsgPath


class PathManager:
    def __init__(self):
        # message sent to path follower
        self.path = MsgPath()
        # pointers to previous, current, and next waypoints
        self.ptr_previous = 0
        self.ptr_current = 1
        self.ptr_next = 2
        self.num_waypoints = 0
        self.halfspace_n = np.inf * np.ones((3,1))
        self.halfspace_r = np.inf * np.ones((3,1))
        # state of the manager state machine
        self.manager_state = 1
        self.manager_requests_waypoints = True
        self.dubins_path = DubinsParameters()

    def update(self, waypoints, radius, state):
        if waypoints.num_waypoints == 0:
            self.manager_requests_waypoints = True
        if self.manager_requests_waypoints is True \
                and waypoints.flag_waypoints_changed is True:
            self.manager_requests_waypoints = False
        if waypoints.type == 'straight_line':
            self.line_manager(waypoints, state)
        elif waypoints.type == 'fillet':
            self.fillet_manager(waypoints, radius, state)
        elif waypoints.type == 'dubins':
            self.dubins_manager(waypoints, radius, state)
        else:
            print('Error in Path Manager: Undefined waypoint type.')
        return self.path

    def line_manager(self, waypoints, state):
        mav_pos = np.array([[state.north, state.east, -state.altitude]]).T
        # if the waypoints have changed, update the waypoint pointer
        
        ##### TODO ######
        # Use functions - self.initialize_pointers(), self.construct_line()
        # self.inHalfSpace(mav_pos), self.increment_pointers(), self.construct_line()

        # Use variables - self.ptr_current, self.manager_requests_waypoints,
        # waypoints.__, radius

    def fillet_manager(self, waypoints, radius, state):
        mav_pos = np.array([[state.north, state.east, -state.altitude]]).T
        # if the waypoints have changed, update the waypoint pointer

        ##### TODO ######
        # Use functions - self.initialize_pointers(), self.construct_fillet_line(),
        # self.inHalfSpace(), self.construct_fillet_circle(), self.increment_pointers()

        # Use variables self.num_waypoints, self.manager_state, self.ptr_current
        # self.manager_requests_waypoints, waypoints.__, radius
      

    def dubins_manager(self, waypoints, radius, state):
        mav_pos = np.array([[state.north, state.east, -state.altitude]]).T
        # if the waypoints have changed, update the waypoint pointer

        ##### TODO #####
        # Use functions - self.initialize_pointers(), self.dubins_path.update(),
        # self.construct_dubins_circle_start(), self.construct_dubins_line(),
        # self.inHalfSpace(), self.construct_dubins_circle_end(), self.increment_pointers(),

        # Use variables - self.num_waypoints, self.dubins_path, self.ptr_current,
        # self.ptr_previous, self.manager_state, self.manager_requests_waypoints,
        # waypoints.__, radius


    def initialize_pointers(self):
        if self.num_waypoints >= 3:
            ##### TODO #####
            self.ptr_previous = 0
            self.ptr_current = 0
            self.ptr_next = 0
        else:
            print('Error Path Manager: need at least three waypoints')

    def increment_pointers(self):
        ##### TODO #####
        self.ptr_previous = 0
        self.ptr_current = 0
        self.ptr_next = 0

    def construct_line(self, waypoints):
        previous = waypoints.ned[:, self.ptr_previous:self.ptr_previous+1]
        ##### TODO #####
        # current = ?
        # next = ?
       
        # update halfspace variables
        # self.halfspace_n =
        # self.halfspace_r = 
        
        # Update path variables
        # self.path.__ =

    def construct_fillet_line(self, waypoints, radius):
        previous = waypoints.ned[:, self.ptr_previous:self.ptr_previous+1]
        ##### TODO #####
        # current = ?
        # next = ?

        # update halfspace variables
        # self.halfspace_n =
        # self.halfspace_r = 
        
        # Update path variables
        # self.path.__ =

    def construct_fillet_circle(self, waypoints, radius):
        previous = waypoints.ned[:, self.ptr_previous:self.ptr_previous+1]
        ##### TODO #####
        # current = ?
        # next = ?

        # update halfspace variables
        # self.halfspace_n =
        # self.halfspace_r = 
        
        # Update path variables
        # self.path.__ =

    def construct_dubins_circle_start(self, waypoints, dubins_path):
        ##### TODO #####
        # update halfspace variables
        # self.halfspace_n =
        # self.halfspace_r = 
        
        # Update path variables
        # self.path.__ =
        pass

    def construct_dubins_line(self, waypoints, dubins_path):
        ##### TODO #####
        # update halfspace variables
        # self.halfspace_n =
        # self.halfspace_r = 
        
        # Update path variables
        # self.path.__ =
        pass

    def construct_dubins_circle_end(self, waypoints, dubins_path):
        ##### TODO #####
        # update halfspace variables
        # self.halfspace_n =
        # self.halfspace_r = 
        
        # Update path variables
        # self.path.__ =
        pass

    def inHalfSpace(self, pos):
        if (pos-self.halfspace_r).T @ self.halfspace_n >= 0:
            return True
        else:
            return False

