"""
msg_map
    - messages type for map of the world
    
part of mavsim_python
    - Beard & McLain, PUP, 2012
    - Last update:
        4/10/2019 - RWB
"""
import numpy as np
import parameters.planner_parameters as PLAN


class MsgWorldMap:
    def __init__(self):
        # flag to indicate if the map has changed
        self.flag_map_changed = 0
        # the city is of size (width)x(width)
        self.city_width = PLAN.city_width
        # number of blocks in city
        self.num_city_blocks = PLAN.num_blocks
        # percent of block that is street.
        self.street_width = PLAN.city_width / PLAN.num_blocks * PLAN.street_width
        # maximum height of buildings
        self.building_max_height = PLAN.building_height
        # an array of building heights
        self.building_height = PLAN.building_height * np.random.rand(PLAN.num_blocks, PLAN.num_blocks)
        # the width of the buildings (all the same)
        self.building_width = PLAN.city_width / PLAN.num_blocks * (1 - PLAN.street_width)
        # north coordinate of center of buildings
        self.building_north = np.zeros((1,PLAN.num_blocks))
        for i in range(PLAN.num_blocks):
            self.building_north[0, i] = 0.5 * (PLAN.city_width / PLAN.num_blocks) * (2 * i + 1)
        # east coordinate of center of buildings
        self.building_east = np.copy(self.building_north)
