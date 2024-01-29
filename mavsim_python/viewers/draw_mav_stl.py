"""
mavsim_python: drawing tools
    - Beard & McLain, PUP, 2012
    - Update history:
        4/15/2019 - BGM
        7/13/2023 - RWB
        1/16/2024 - RWB
"""
import numpy as np
import pyqtgraph.opengl as gl
from tools.rotations import euler_to_rotation
from stl import mesh

class DrawMav:
    def __init__(self, state, window, scale=10):
        """
        Draw the MAV.

        The input to this function is a (message) class with properties that define the state.
        The following properties are assumed:
            state.north  # north position
            state.east  # east position
            state.altitude   # altitude
            state.phi  # roll angle
            state.theta  # pitch angle
            state.psi  # yaw angle
        """
        self.unit_length = scale
        mav_position = np.array([[state.north], [state.east], [-state.altitude]])  # NED coordinates
        # attitude of mav as a rotation matrix R from body to inertial
        R_bi = euler_to_rotation(state.phi, state.theta, state.psi)
        # convert North-East Down to East-North-Up for rendering
        self.R_ned = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])

        # get stl mesh
        stl_mesh = mesh.Mesh.from_file('viewers/aircraft1.stl')
        self.mav_points = self.unit_length*stl_mesh.points.reshape(-1, 3)
        self.mav_points = self.rotate_points(self.mav_points, np.diag([-1, -1, 1])@self.R_ned.T)
        self.mav_faces = np.arange(self.mav_points.shape[0]).reshape(-1, 3)
        self.mav_body = self.add_object(
            self.mav_points,
            self.mav_faces,
            R_bi,
            mav_position)
        window.addItem(self.mav_body)  # add mav to plot     

    def update(self, state):
        mav_position = np.array([[state.north], [state.east], [-state.altitude]])  # NED coordinates
        # attitude of mav as a rotation matrix R from body to inertial
        R_bi = euler_to_rotation(state.phi, state.theta, state.psi)
        self.mav_body = self.update_object(
            self.mav_body,
            self.mav_points,
            self.mav_faces,
            R_bi,
            mav_position)
        
    def add_object(self, points, faces, R, position):
        rotated_points = self.rotate_points(points, R)
        translated_points = self.translate_points(rotated_points, position)
        translated_points = self.rotate_points(translated_points, self.R_ned)
        object = gl.GLMeshItem(
            vertexes=translated_points, 
            faces=faces,
            drawEdges=True,  # draw edges between mesh elements
            smooth=False,  # speeds up rendering
            computeNormals=False)  # speeds up rendering
        return object

    def update_object(self, object, points, faces, R, position):
        rotated_points = self.rotate_points(points, R)
        translated_points = self.translate_points(rotated_points, position)
        translated_points = self.rotate_points(translated_points, self.R_ned)
        object.setMeshData(vertexes=translated_points, faces=faces)
        return object
    
    def rotate_points(self, points, R):
        "Rotate points by the rotation matrix R"
        rotated_points = points @ R.T
        return rotated_points

    def translate_points(self, points, translation):
        "Translate points by the vector translation"
        translated_points = points + np.dot(np.ones([points.shape[0],1]),translation.T)
        return translated_points
