# rrt dubins path planner for mavsim_python
#
# mavsim_python
#     - Beard & McLain, PUP, 2012
#     - Last updated:
#         4/16/2019 - RWB
import numpy as np
from message_types.msg_waypoints import MsgWaypoints
from planning.dubins_parameters import DubinsParameters
from viewers.planner_viewer import PlannerViewer


class RRTDubins:
    def __init__(self, app, show_planner=True):
        self.segment_length = 450  # standard length of path segments
        self.dubins_path = DubinsParameters()
        # initialize Qt gui application and window
        self.show_planner = show_planner
        if show_planner:
            self.planner_viewer = PlannerViewer(app)

    def update(self, start_pose, end_pose, Va, world_map, radius):
        self.segment_length = 4 * radius
        tree = MsgWaypoints()
        tree.type = 'dubins'

        ##### TODO #####
        # add the start pose to the tree
        
        # check to see if start_pose connects directly to end_pose
       
        # find path with minimum cost to end_node
        # waypoints_not_smooth = findMinimumPath()
        # waypoints = self.smoothPath()
        waypoints_not_smooth = MsgWaypoints()
        waypoints = MsgWaypoints()
        if self.show_planner:
            self.planner_viewer.draw_tree_and_map(world_map, tree, waypoints_not_smooth, waypoints, radius, self.dubins_path)
        return waypoints

    def extendTree(self, tree, end_pose, Va, world_map, radius):
        # extend tree by randomly selecting pose and extending tree toward that pose
        
        ##### TODO #####
        flag = None
        return flag

    def collision(self, start_pose, end_pose, world_map, radius):
        # check to see of path from start_pose to end_pose colliding with world_map
        
        ##### TODO #####
        collision_flag = None
        return collision_flag

    def process_app(self):
        self.planner_viewer.process_app()

    def smoothPath(self, waypoints, world_map, radius):
        
        ##### TODO #####
        # smooth the waypoint path
        smooth = [0]  # add the first waypoint
        
        # construct smooth waypoint path
        smooth_waypoints = MsgWaypoints()
        
        return smooth_waypoints


def findMinimumPath(tree, end_pose):
    # find the lowest cost path to the end node

    ##### TODO #####
    # find nodes that connect to end_node
    connecting_nodes = []
    
    # find minimum cost last node
    idx = 0
    
    # construct lowest cost path order
    path = []

    # construct waypoint path
    waypoints = MsgWaypoints()
   
    return waypoints


def distance(start_pose, end_pose):
    # compute distance between start and end pose
    ##### TODO #####
    d = 0
    return d


def heightAboveGround(world_map, point):
    # find the altitude of point above ground level
    ###### TODO ######
    h_agl = 0
    return h_agl


def randomPose(world_map, pd):
    # generate a random pose
    ###### TODO ######
    pn = 0
    pe = 0
    chi = 0
    pose = np.array([[pn], [pe], [pd], [chi]])
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
