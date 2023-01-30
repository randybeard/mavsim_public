"""
mavsim_python: drawing tools
    - Beard & McLain, PUP, 2012
    - Update history:
        4/15/2019 - BGM
"""

import numpy as np
import pyqtgraph.opengl as gl


class DrawPath:
    def __init__(self, path, color, window):
        self.color = color
        if path.type == 'line':
            scale = 1000
            points = straight_line_points(path, scale)
        elif path.type == 'orbit':
            points = orbit_points(path)
        path_color = np.tile(color, (points.shape[0], 1))
        self.path_plot_object = gl.GLLinePlotItem(pos=points,
                                                  color=path_color,
                                                  width=1,
                                                  antialias=True,
                                                  mode='line_strip')
        window.addItem(self.path_plot_object)

    def update(self, path, color):
        if path.type == 'line':
            scale = 1000
            points = straight_line_points(path, scale)
        elif path.type == 'orbit':
            points = orbit_points(path)
        path_color = np.tile(color, (points.shape[0], 1))
        self.path_plot_object.setData(pos=points, color=path_color)

def straight_line_points(path, scale):
    points = np.array([[path.line_origin.item(0),
                        path.line_origin.item(1),
                        path.line_origin.item(2)],
                        [path.line_origin.item(0) + scale * path.line_direction.item(0),
                        path.line_origin.item(1) + scale * path.line_direction.item(1),
                        path.line_origin.item(2) + scale * path.line_direction.item(2)]])
    # convert North-East Down to East-North-Up for rendering
    R = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])
    points = points @ R.T
    return points


def orbit_points(path):
    N = 10
    theta = 0
    theta_list = [theta]
    while theta < 2*np.pi:
        theta += 0.1
        theta_list.append(theta)
    points = np.array([[path.orbit_center.item(0) + path.orbit_radius,
                        path.orbit_center.item(1),
                        path.orbit_center.item(2)]])
    for angle in theta_list:
        new_point = np.array([[path.orbit_center.item(0) + path.orbit_radius * np.cos(angle),
                                path.orbit_center.item(1) + path.orbit_radius * np.sin(angle),
                                path.orbit_center.item(2)]])
        points = np.concatenate((points, new_point), axis=0)
    # convert North-East Down to East-North-Up for rendering
    R = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])
    points = points @ R.T
    return points
