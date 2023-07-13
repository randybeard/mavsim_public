"""
mavsim: camera viewer (for chapter 13)
    - Beard & McLain, PUP, 2012
    - Update history:
        4/15/2019 - RWB
        3/31/2022 - RWB
"""
import numpy as np
import matplotlib.pyplot as plt
import parameters.camera_parameters as CAM
from tools.rotations import Euler2Rotation
from message_types.msg_camera import MsgCamera

class Camera:
    def __init__(self):
        self.target_points = self._getPoints()
        self.pixels = MsgCamera()
        self.projected_points = []

    def updateProjectedPoints(self, state, target_position):
        mav_position = np.array([[state.north], [state.east], [-state.altitude]])  # NED coordinates
        # attitude of mav as a rotation matrix R from body to inertial
        R = Euler2Rotation(state.phi, state.theta, state.psi)  # R_b^i
        Rgim = Euler2Rotation(0, state.camera_el, state.camera_az)  # R_g^b
        Rcam = np.array([[0, 1, 0], [0, 0, 1], [1, 0, 0]])  # R_g^c
        Rtarget = np.eye(3)  # R_t^i
        points = self._rotatePoints(self.target_points, Rtarget)  # rotate target to inertial frame
        points = self._translatePoints(points, target_position - mav_position)  # translate to camera frame
        points = self._rotatePoints(points, Rgim.T @ R.T)  # rotate into the gimbal frame
        points = self._rotatePoints(points, Rcam)  # rotate into camera frame
        # rotate and translate points defining mav
        self.projected_points = self._projectOnCameraPlane(points)
        # set pixel value as the first projected point
        self.pixels.pixel_x = self.projected_points[0, 0]
        self.pixels.pixel_y = self.projected_points[1, 0]

    def getPixels(self):
        return self.pixels

    def getProjectedPoints(self):
        return self.projected_points

    def _projectOnCameraPlane(self, points):
        m, n = points.shape
        projected_points = np.zeros((2, n))
        for i in range(0, n):
            projected_points[0, i] = CAM.f * points[0, i] / points[2, i]
            projected_points[1, i] = CAM.f * points[1, i] / points[2, i]
        return projected_points

    def _rotatePoints(self, points, R):
        "Rotate points by the rotation matrix R"
        rotated_points = R @ points
        return rotated_points

    def _translatePoints(self, points, translation):
        "Translate points by the vector translation"
        translated_points = points + np.dot(translation, np.ones([1, points.shape[1]]))
        return translated_points

    def _getPoints(self):
        # Points that define the target, and the colors of the triangular mesh
        length = 100
        # points are in NED coordinates
        #   define the points on the target (3D box)
        points = np.array([
            [length / 2, length / 2, 0],  # point 0
            [length / 2, -length / 2, 0],  # point 1
            [-length / 2, -length / 2, 0],  # point 2
            [-length / 2, length / 2, 0],  # point 3
            [length / 2, length / 2, -length],  # point 4
            [length / 2, -length / 2, -length],  # point 5
            [-length / 2, -length / 2, -length],  # point 6
            [-length / 2, length / 2, -length],  # point 7
        ]).T
        return points
