import numpy as np
from math import sin, cos
import sys

sys.path.append('..')
from message_types.msg_autopilot import MsgAutopilot
from tools.wrap import wrap


class PathFollower:
    def __init__(self):
        self.chi_inf =   # approach angle for large distance from straight-line path
        self.k_path =  #0.05  # proportional gain for straight-line path following
        self.k_orbit = # 10.0  # proportional gain for orbit following
        self.gravity = 9.8
        self.autopilot_commands = MsgAutopilot()  # message sent to autopilot

    def update(self, path, state):
        if path.type == 'line':
            self._follow_straight_line(path, state)
        elif path.type == 'orbit':
            self._follow_orbit(path, state)
        return self.autopilot_commands

    def _follow_straight_line(self, path, state):
        self.autopilot_commands.airspeed_command = 
        # course command
        self.autopilot_commands.course_command = 
        # altitude command
        self.autopilot_commands.altitude_command =
        # feedforward roll angle for straight line is zero
        self.autopilot_commands.phi_feedforward = 

    def _follow_orbit(self, path, state):
        if path.orbit_direction == 'CW':
            direction = 1.0
        else:
            direction = -1.0
        # airspeed command
        self.autopilot_commands.airspeed_command = 
        # distance from orbit center
        d = 
        # compute wrapped version of angular position on orbit
        varphi = 
        # compute normalized orbit error
        orbit_error = 
        # course command
        self.autopilot_commands.course_command = 
        # altitude command
        self.autopilot_commands.altitude_command = 
        # roll feedforward command
        if orbit_error < 10:
            self.autopilot_commands.phi_feedforward =
        else:
            self.autopilot_commands.phi_feedforward = 



