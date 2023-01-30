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
from viewers.planner_viewer import PlannerViewer

class RRTStraightLine:
    def __init__(self, app, show_planner=True):
        self.segment_length = 300 # standard length of path segments
        self.show_planner = show_planner
        if show_planner:
            self.planner_viewer = PlannerViewer(app)

    def update(self, start_pose, end_pose, Va, world_map, radius):
        tree = MsgWaypoints()
        #tree.type = 'straight_line'
        tree.type = 'fillet'

        ###### TODO ######
        # add the start pose to the tree
        
        # check to see if start_pose connects directly to end_pose
        
        # find path with minimum cost to end_node
        # waypoints_not_smooth = find_minimum_path()
        # waypoints = smooth_path()
        waypoints = MsgWaypoints()
        return waypoints

    def extend_tree(self, tree, end_pose, Va, world_map):
        # extend tree by randomly selecting pose and extending tree toward that pose
        
        ###### TODO ######
        flag = None
        return flag
        
    def process_app(self):
        self.planner_viewer.process_app()

def smooth_path(waypoints, world_map):

    ##### TODO #####
    # smooth the waypoint path
    smooth = [0]  # add the first waypoint
    
    # construct smooth waypoint path
    smooth_waypoints = MsgWaypoints()

    return smooth_waypoints


def find_minimum_path(tree, end_pose):
    # find the lowest cost path to the end node

    ##### TODO #####
    # find nodes that connect to end_node
    connecting_nodes = []
    
    # find minimum cost last node
    idx = 0

    # construct lowest cost path order
    path = []  # last node that connects to end node
    
    # construct waypoint path
    waypoints = MsgWaypoints()
    return waypoints


def random_pose(world_map, pd):
    # generate a random pose

    ##### TODO #####
    pn = 0
    pe = 0
    pose = np.array([[pn], [pe], [pd]])
    return pose


def distance(start_pose, end_pose):
    # compute distance between start and end pose

    ##### TODO #####
    d = 0
    return d


def collision(start_pose, end_pose, world_map):
    # check to see of path from start_pose to end_pose colliding with map
    
    ###### TODO ######
    collision_flag = None
    return collision_flag


def height_above_ground(world_map, point):
    # find the altitude of point above ground level
    
    ##### TODO #####
    h_agl = 0
    return h_agl

def points_along_path(start_pose, end_pose, N):
    # returns points along path separated by Del
    points = None
    return points


def column(A, i):
    # extracts the ith column of A and return column vector
    tmp = A[:, i]
    col = tmp.reshape(A.shape[0], 1)
    return col