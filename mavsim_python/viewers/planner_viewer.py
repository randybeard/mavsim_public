"""
mavsim_python: world viewer (for chapter 12)
    - Beard & McLain, PUP, 2012
    - Update history:
        4/15/2019 - RWB
        3/30/2022 - RWB
        7/13/2023 - RWB
"""
import numpy as np
#from message_types.msg_waypoints import MsgWaypoints
from viewers.draw_waypoints import DrawWaypoints
from viewers.draw_map import DrawMap
from planners.dubins_parameters import DubinsParameters
#import pyqtgraph as pg
import pyqtgraph.opengl as gl


class PlannerViewer:
    def __init__(self, app):
        self.scale = 2500
        # initialize Qt gui application and window
        self.app = app  # initialize QT
        self.window = gl.GLViewWidget()  # initialize the view object
        self.window.setWindowTitle('RRT Tree Viewer')
        self.window.setGeometry(500, 0, 500, 500)  # args: upper_left_x, upper_right_y, width, height
        grid = gl.GLGridItem() # make a grid to represent the ground
        grid.scale(self.scale/20, self.scale/20, self.scale/20) # set the size of the grid (distance between each line)
        self.window.addItem(grid) # add grid to viewer
        center = self.window.cameraPosition()
        center.setX(1000)
        center.setY(1000)
        center.setZ(0)
        self.window.setCameraPosition(pos=center, 
                                      distance=self.scale, 
                                      elevation=50, 
                                      azimuth=-90)
        self.window.setBackgroundColor('k')  # set background color to black
        # self.window.resize(*(4000, 4000))  # not sure how to resize window
        self.window.show()  # display configured window
        self.window.raise_()  # bring window to the front
        self.R = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])

    def draw_tree_and_map(self, world_map, tree, waypoints, smoothed_waypoints,
                           radius, dubins_path=DubinsParameters()):
        blue = np.array([[30, 144, 255, 255]])/255.
        red = np.array([[204, 0, 0]])/255.
        green = np.array([[0, 153, 51]])/255.
        self.window.clear()
        DrawMap(world_map, self.window)
        # draw things to the screen
        for i in range(1, tree.num_waypoints):
            if dubins_path == None:
                points = self.get_straight_line_points(tree, i)
            else:
                points = self.get_dubins_points(tree,i,radius,dubins_path)
            tree_color = np.tile(green, (points.shape[0], 1))
            tree_plot_object = gl.GLLinePlotItem(pos=points,
                                                color=tree_color,
                                                width=2,
                                                antialias=True,
                                                mode='line_strip')
            self.window.addItem(tree_plot_object)
            DrawWaypoints(waypoints, radius, blue, self.window)
            DrawWaypoints(smoothed_waypoints, radius, red, self.window)

    def get_straight_line_points(self, tree, i):
        points = self.R @ tree.ned
        parent = int(tree.parent.item(i))
        points = np.concatenate((self.column(points, i).T, self.column(points, parent).T), axis=0)
        return points

    def get_dubins_points(self, tree, i, radius, dubins_path):
        parent = int(tree.parent.item(i))
        dubins_path.update(self.column(tree.ned, parent), tree.course[parent],
                            self.column(tree.ned, i), tree.course[i], radius)
        points = dubins_path.compute_points()
        points = points @ self.R.T
        return points

    def column(self, A, i):
        # extracts the ith column of A and return column vector
        tmp = A[:, i]
        col = tmp.reshape(A.shape[0], 1)
        return col
    
    def process_app(self):
        self.app.processEvents()


