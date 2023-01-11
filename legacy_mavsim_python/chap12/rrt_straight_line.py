# rrt straight line path planner for mavsim_python
#
# mavsim_python
#     - Beard & McLain, PUP, 2012
#     - Last updated:
#         4/3/2019 - Brady Moon
#         4/11/2019 - RWB
#         3/31/2020 - RWB
import numpy as np
from message_types.msg_waypoints import MsgWaypoints
from chap11.draw_waypoints import DrawWaypoints
from chap12.draw_map import DrawMap
import pyqtgraph as pg
import pyqtgraph.opengl as gl


class RRTStraightLine:
    def __init__(self):
        self.segment_length = 300 # standard length of path segments
        self.plot_window = []
        self.plot_app = []

    def update(self, start_pose, end_pose, Va, world_map, radius):
        #generate tree
        tree = MsgWaypoints()
        #tree.type = 'straight_line'
        tree.type = 'fillet'
        # add the start pose to the tree
        
        # check to see if start_pose connects directly to end_pose
       
        # find path with minimum cost to end_node
        waypoints_not_smooth = #find_minimum_path()
        waypoints = #smooth_path()
        return waypoints

    def extend_tree(self, tree, end_pose, Va, world_map):
        # extend tree by randomly selecting pose and extending tree toward that pose
        flag = 
        return flag

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
        #self.plot_window.raise_() # bring window to the front

        blue = np.array([[30, 144, 255, 255]])/255.
        red = np.array([[204, 0, 0]])/255.
        green = np.array([[0, 153, 51]])/255.
        DrawMap(world_map, self.plot_window)
        DrawWaypoints(waypoints, radius, blue, self.plot_window)
        DrawWaypoints(smoothed_waypoints, radius, red, self.plot_window)
        draw_tree(tree, green, self.plot_window)
        # draw things to the screen
        self.plot_app.processEvents()


def smooth_path(waypoints, world_map):
    # smooth the waypoint path
    smooth = [0]  # add the first waypoint

    # construct smooth waypoint path
    smooth_waypoints = MsgWaypoints()

    return smooth_waypoints


def find_minimum_path(tree, end_pose):
    # find the lowest cost path to the end node
    # find nodes that connect to end_node
    connecting_nodes = []
   
    # find minimum cost last node
    idx = 

    # construct lowest cost path order
    path = 
    
    # construct waypoint path
    waypoints = MsgWaypoints()
   
    return waypoints


def random_pose(world_map, pd):
    # generate a random pose
    pose =
    return pose


def distance(start_pose, end_pose):
    # compute distance between start and end pose
    d = 
    return d


def collision(start_pose, end_pose, world_map):
    # check to see of path from start_pose to end_pose colliding with map
    collision_flag = 

    return collision_flag


def height_above_ground(world_map, point):
    # find the altitude of point above ground level
    point_height = 
    map_height = 
    h_agl = 
    return h_agl


def draw_tree(tree, color, window):
    R = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])
    points = R @ tree.ned
    for i in range(points.shape[1]):
        line_color = np.tile(color, (2, 1))
        parent = int(tree.parent.item(i))
        line_pts = np.concatenate((column(points, i).T, column(points, parent).T), axis=0)
        line = gl.GLLinePlotItem(pos=line_pts,
                                 color=line_color,
                                 width=2,
                                 antialias=True,
                                 mode='line_strip')
        window.addItem(line)


def points_along_path(start_pose, end_pose, N):
    # returns points along path separated by Del
    points = 
    return points


def column(A, i):
    # extracts the ith column of A and return column vector
    tmp = A[:, i]
    col = tmp.reshape(A.shape[0], 1)
    return col