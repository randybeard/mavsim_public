# rrt dubins path planner for mavsim_python
import numpy as np
from message_types.msg_waypoints import MsgWaypoints
from planners.dubins_parameters import DubinsParameters


class RRTDubins:
    def __init__(self):
        self.segment_length = 450  # standard length of path segments
        self.dubins_path = DubinsParameters()

    def update(self, start_pose, end_pose, Va, world_map, radius):
        self.segment_length = 4 * radius
        tree = MsgWaypoints()
        tree.type = 'dubins'
        waypoints_not_smooth = MsgWaypoints()
        waypoints = MsgWaypoints()        

        ##### TODO #####
        # add the start pose to the tree
        
        # check to see if start_pose connects directly to end_pose
       
        # find path with minimum cost to end_node
        # waypoints_not_smooth = findMinimumPath()
        # waypoints = self.smoothPath()
        self.waypoint_not_smooth = waypoints_not_smooth
        self.tree = tree
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

    # find nodes that connect to end_node
    connecting_nodes = []
    for i in range(tree.num_waypoints):
        if tree.connect_to_goal.item(i) == 1:
            connecting_nodes.append(i)
    # find minimum cost last node
    idx = np.argmin(tree.cost[connecting_nodes])
    # construct lowest cost path order
    path = [connecting_nodes[idx]]  # last node that connects to end node
    parent_node = tree.parent.item(connecting_nodes[idx])
    while parent_node >= 1:
        path.insert(0, int(parent_node))
        parent_node = tree.parent.item(int(parent_node))
    path.insert(0, 0)
    # construct waypoint path
    waypoints = MsgWaypoints()
    for i in path:
        waypoints.add(column(tree.ned, i),
                      tree.airspeed.item(i),
                      tree.course.item(i),
                      np.inf,
                      np.inf,
                      np.inf)
    waypoints.add(end_pose[0:3],
                  tree.airspeed[-1],
                  end_pose.item(3),
                  np.inf,
                  np.inf,
                  np.inf)
    waypoints.type = tree.type
    return waypoints


def distance(start_pose, end_pose):
    # compute distance between start and end pose
    d = np.linalg.norm(start_pose[0:3] - end_pose[0:3])
    return d


def heightAboveGround(world_map, point):
    # find the altitude of point above ground level
    point_height = -point.item(2)
    tmp = np.abs(point.item(0)-world_map.building_north)
    d_n = np.min(tmp)
    idx_n = np.argmin(tmp)
    tmp = np.abs(point.item(1)-world_map.building_east)
    d_e = np.min(tmp)
    idx_e = np.argmin(tmp)
    if (d_n<world_map.building_width) and (d_e<world_map.building_width):
        map_height = world_map.building_height[idx_n, idx_e]
    else:
        map_height = 0
    h_agl = point_height - map_height
    return h_agl


def randomPose(world_map, pd):
    # generate a random pose
    pn = world_map.city_width * np.random.rand()
    pe = world_map.city_width * np.random.rand()
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
