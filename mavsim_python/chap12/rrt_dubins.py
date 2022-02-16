# rrt dubins path planner for mavsim_python
#
# mavsim_python
#     - Beard & McLain, PUP, 2012
#     - Last updated:
#         4/16/2019 - RWB
import numpy as np
from message_types.msg_waypoints import MsgWaypoints
from chap11.draw_waypoints import DrawWaypoints
from chap12.draw_map import DrawMap
from chap11.dubins_parameters import DubinsParameters
import pyqtgraph as pg
import pyqtgraph.opengl as gl


class RRTDubins:
    def __init__(self):
        self.segment_length = 300  # standard length of path segments
        self.plot_window = []
        self.plot_app = []
        self.dubins_path = DubinsParameters()

    def update(self, start_pose, end_pose, Va, world_map, radius):
        #generate tree
        tree = MsgWaypoints()
        tree.type = 'dubins'
        # add the start pose to the tree
        
        # check to see if start_pose connects directly to end_pose
        
        # find path with minimum cost to end_node
        waypoints_not_smooth = #find_minimum_path()
        waypoints = #smooth_path()

        return waypoints

    def extend_tree(self, tree, end_pose, Va, world_map, radius):
        # extend tree by randomly selecting pose and extending tree toward that pose
        flag = 
        return flag

    def points_along_path(self, Del):
        # returns a list of points along the dubins path
        initialize_points = True
        # points along start circle
        theta_list = []
        if initialize_points:
            points = 
            initialize_points = False
        for angle in theta_list:
            points = 

        # points along straight line
        sig = 0
        while sig <= 1:
            points =
            sig = 

        # points along end circle
    
        theta_list = []
        for angle in theta_list:
            points = 
        return points

    def collision(self, start_pose, end_pose, world_map, radius):
        # check to see of path from start_pose to end_pose colliding with world_map
        collision_flag = 
        return collision_flag

    def plot_map(self, world_map, tree, waypoints, smoothed_waypoints, radius):
        scale = 4000
        # initialize Qt gui application and window
        self.plot_app = pg.QtGui.QApplication([])  # initialize QT
        self.plot_window = gl.GLViewWidget()  # initialize the view object
        self.plot_window.setWindowTitle('World Viewer')
        self.plot_window.setGeometry(0, 0, 1500, 1500)  # args: upper_left_x, upper_right_y, width, height
        grid = gl.GLGridItem() # make a grid to represent the ground
        grid.scale(scale/20, scale/20, scale/20) # set the size of the grid (distance between each line)
        self.plot_window.addItem(grid) # add grid to viewer
        self.plot_window.setCameraPosition(distance=scale, elevation=50, azimuth=-90)
        self.plot_window.setBackgroundColor('k')  # set background color to black
        self.plot_window.show()  # display configured window
        self.plot_window.raise_() # bring window to the front

        blue = np.array([[30, 144, 255, 255]])/255.
        red = np.array([[204, 0, 0]])/255.
        green = np.array([[0, 153, 51]])/255.
        DrawMap(world_map, self.plot_window)
        DrawWaypoints(waypoints, radius, blue, self.plot_window)
        DrawWaypoints(smoothed_waypoints, radius, red, self.plot_window)
        self.draw_tree(radius, green)
        # draw things to the screen
        self.plot_app.processEvents()

    def draw_tree(self, radius, color, window):
        R = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])
        Del = 0.05
        for i in range(1, tree.num_waypoints):
            parent = int(tree.parent.item(i))
            self.dubins_path.update(column(tree.ned, parent), tree.course[parent],
                                    column(tree.ned, i), tree.course[i], radius)
            points = self.points_along_path(Del)
            points = points @ R.T
            tree_color = np.tile(color, (points.shape[0], 1))
            tree_plot_object = gl.GLLinePlotItem(pos=points,
                                                color=tree_color,
                                                width=2,
                                                antialias=True,
                                                mode='line_strip')
            self.plot_window.addItem(tree_plot_object)


def find_minimum_path(tree, end_pose):
    # find the lowest cost path to the end node
    # find nodes that connect to end_node
    connecting_nodes = 

    # find minimum cost last node
    idx = 

    # construct lowest cost path order
    path =   # last node that connects to end node

    # construct waypoint path
    waypoints = MsgWaypoints()

    return waypoints


def smooth_path(waypoints, world_map, radius):
    # smooth the waypoint path
    smooth = [0]  # add the first waypoint

    # construct smooth waypoint path
    smooth_waypoints = MsgWaypoints()
    
    return smooth_waypoints


def distance(start_pose, end_pose):
    # compute distance between start and end pose
    d = 
    return d


def height_above_ground(world_map, point):
    # find the altitude of point above ground level
    point_height = 
    if (d_n<world_map.building_width) and (d_e<world_map.building_width):
        map_height = 
    else:
        map_height = 
    h_agl = 
    return h_agl


def random_pose(world_map, pd):
    # generate a random pose
    pose = 
    return pose


def mod(x):
    # force x to be between 0 and 2*pi
    while x < 0:
        x += 2*np.pi
    while x > 2*np.pi:
        x -= 2*np.pi
    return x


def column(A, i):
    # extracts the ith column of A and return column vector
    tmp = A[:, i]
    col = tmp.reshape(A.shape[0], 1)
    return col