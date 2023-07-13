import numpy as np
import parameters.aerosonde_parameters as MAV

# size of the waypoint array used for the path planner.  This is the
# maximum number of waypoints that might be transmitted to the path
# manager.
size_waypoint_array = 100

# airspeed commanded by planner
Va0 = MAV.u0

# max possible roll angle
phi_max = np.radians(25.)

# minimum turn radius
R_min = Va0**2. / MAV.gravity / np.tan(phi_max)
#print(R_min)

# create random city map
city_width      = 2000.  # the city is of size (width)x(width)
building_height = 300.   # maximum height of buildings
num_blocks      = 5    # number of blocks in city
street_width    = .8   # percent of block that is street.



