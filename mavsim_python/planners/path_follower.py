import numpy as np
from math import sin, cos
from message_types.msg_autopilot import MsgAutopilot
from tools.wrap import wrap


class PathFollower:
    def __init__(self):
        ##### TODO #####
        self.chi_inf = 0  # approach angle for large distance from straight-line path
        self.k_path = 0  # path gain for straight-line path following
        self.k_orbit = 0  # path gain for orbit following
        self.gravity = 0
        self.autopilot_commands = MsgAutopilot()  # message sent to autopilot

    def update(self, path, state):
        if path.type == 'line':
            self._follow_straight_line(path, state)
        elif path.type == 'orbit':
            self._follow_orbit(path, state)
        return self.autopilot_commands

    def _follow_straight_line(self, path, state):
        ##### TODO #####
        #airspeed command
        self.autopilot_commands.airspeed_command = 0

        # course command
        self.autopilot_commands.course_command = 0

        # altitude command
        self.autopilot_commands.altitude_command = 0

        # feedforward roll angle for straight line is zero
        self.autopilot_commands.phi_feedforward = 0

    def _follow_orbit(self, path, state):
        ##### TODO #####
        # airspeed command
        self.autopilot_commands.airspeed_command = 0

        # course command
        self.autopilot_commands.course_command = 0

        # altitude command
        self.autopilot_commands.altitude_command = 0
        
        # roll feedforward command
        self.autopilot_commands.phi_feedforward = 0




